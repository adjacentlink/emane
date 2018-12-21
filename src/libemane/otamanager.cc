/*
 * Copyright (c) 2013-2017 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of DRS CenGen, LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "otamanager.h"
#include "otauser.h"
#include "logservice.h"
#include "controlmessageserializer.h"
#include "otaexception.h"
#include "eventservice.h"
#include "otaheader.pb.h"
#include "event.pb.h"
#include "socketexception.h"

#include "emane/net.h"
#include "emane/utils/threadutils.h"
#include "emane/controls/otatransmittercontrolmessage.h"
#include "emane/controls/serializedcontrolmessage.h"

#include <sstream>
#include <algorithm>
#include <uuid.h>

namespace
{
  struct PartInfo
  {
    std::uint8_t u8More_; /**< More parts to follow*/
    std::uint32_t u32Offset_; /**< Offset of payload */
    std::uint32_t u32Size_;     /**< Part size */
  } __attribute__((packed));

  std::vector<uint8_t> bufferFromVectorIO(size_t size,
                                          size_t & index,
                                          size_t & offset,
                                          const EMANE::Utils::VectorIO & vectorIO)
  {
    std::vector<uint8_t> buf{};

    size_t targetBytes{size};

    while(targetBytes)
      {
        size_t available{vectorIO[index].iov_len - offset};

        if(available)
          {
            if(available >= targetBytes)
              {
                buf.insert(buf.end(),
                           &reinterpret_cast<uint8_t *>(vectorIO[index].iov_base)[offset],
                           &reinterpret_cast<uint8_t *>(vectorIO[index].iov_base)[offset+targetBytes]);

                offset += targetBytes;

                targetBytes = 0;
              }
            else
              {
                buf.insert(buf.end(),
                           &reinterpret_cast<uint8_t *>(vectorIO[index].iov_base)[offset],
                           &reinterpret_cast<uint8_t *>(vectorIO[index].iov_base)[offset] + available);

                targetBytes -= available;

                ++index;

                offset = 0;
              }
          }
      }

    return buf;
  }
}

EMANE::OTAManager::OTAManager():
  bOpen_(false),
  otaMTU_{},
  eventStatisticPublisher_{"OTAChannel"},
  u64SequenceNumber_{},
  lastPartCheckTime_{}
{
  uuid_clear(uuid_);
}

EMANE::OTAManager::~OTAManager()
{
  if(bOpen_)
    {
      ThreadUtils::cancel(thread_);

      thread_.join();
    }
}

void EMANE::OTAManager::setStatPacketCountRowLimit(size_t rows)
{
  otaStatisticPublisher_.setRowLimit(rows);
}

void EMANE::OTAManager::setStatEventCountRowLimit(size_t rows)
{
  eventStatisticPublisher_.setRowLimit(rows);
}

void EMANE::OTAManager::sendOTAPacket(NEMId id,
                                      const DownstreamPacket & pkt,
                                      const ControlMessages & msgs) const
{
  // get the pkt info
  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  // set of optional additional transmitters (AT)
  Controls::OTATransmitters otaTransmitters{};

  auto eventSerializations = pkt.getEventSerializations();

  EMANEMessage::Event::Data data;

  if(!eventSerializations.empty())
    {
      NEMId targetNEMId;
      EventId eventId;
      Serialization serialization;

      for(const auto & entry : eventSerializations)
        {
          std::tie(targetNEMId,
                   eventId,
                   serialization) = entry;

          // process any local event
          EventServiceSingleton::instance()->processEventMessage(targetNEMId,
                                                                 eventId,
                                                                 serialization,
                                                                 id);

          if(bOpen_)
            {
              auto pSerialization = data.add_serializations();

              pSerialization->set_nemid(targetNEMId);

              pSerialization->set_eventid(eventId);

              pSerialization->set_data(serialization);
            }
        }
    }

  for(const auto & pMessage : msgs)
    {
      if(pMessage->getId() == Controls::OTATransmitterControlMessage::IDENTIFIER)
        {
          const auto pTransmitterControlMessage =
            reinterpret_cast<const Controls::OTATransmitterControlMessage *>(pMessage);

          otaTransmitters = pTransmitterControlMessage->getOTATransmitters();
        }
    }

  /*
   * UpstreamPacket data is shared (reference counted).  The same
   * packet can be used in multiple calls to OTAUser::processOTAPacket
   * since the resulting action is to enqueue a referenced counted
   * copy on each NEM queue. Each copy will share the same packet data
   * but have unique index counters used for stripping packet data.
   */
  if(nemUserMap_.size() > 1)
    {
      auto now = Clock::now();

      UpstreamPacket upstreamPacket({pktInfo.getSource(),
            pktInfo.getDestination(),
            pktInfo.getPriority(),
            now,
            uuid_},
        pkt.getVectorIO());

      // bounce a copy of the pkt back up to our local NEM stack(s)
      for(NEMUserMap::const_iterator iter = nemUserMap_.begin(), end = nemUserMap_.end();
          iter != end;
          ++iter)
        {
          if(iter->first == id)
            {
              // skip our own transmisstion
            }
          else if(otaTransmitters.count(iter->first) > 0)
            {
              // skip NEM(s) in the additional transmitter set (ATS)
            }
          else
            {
              iter->second->processOTAPacket(upstreamPacket,ControlMessages());
            }
        }
    }

  // send the packet to additional OTAManagers using OTA multicast transport
  if(bOpen_)
    {
      std::string sEventSerialization{};

      if(!eventSerializations.empty())
        {
          if(!data.SerializeToString(&sEventSerialization))
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "OTAManager sendOTAPacket unable to serialize attached event data src:%hu dst:%hu",
                                      pktInfo.getSource(),
                                      pktInfo.getDestination());
            }
        }

      ControlMessageSerializer controlMessageSerializer{msgs};

      // create an ota message to carry the packet_info, and variable ctrl data len only
      // total message length with data payload is defined below
      EMANEMessage::OTAHeader otaheader;

      size_t totalSizeBytes = pkt.length() +
        controlMessageSerializer.getLength() +
        sEventSerialization.size();

      otaheader.set_source(pktInfo.getSource());
      otaheader.set_destination(pktInfo.getDestination());
      otaheader.set_sequence(++u64SequenceNumber_);
      otaheader.set_uuid(reinterpret_cast<const char *>(uuid_),sizeof(uuid_));

      auto pPayloadInfo = otaheader.mutable_payloadinfo();

      pPayloadInfo->set_datalength(pkt.length());
      pPayloadInfo->set_controllength(controlMessageSerializer.getLength());
      pPayloadInfo->set_eventlength(sEventSerialization.size());

      // vector hold everything to be transmitted except the OTAHeader
      Utils::VectorIO stagingVectorIO{};
      size_t stagingIndex{};
      size_t stagingOffset{};

      if(!sEventSerialization.empty())
        {
          stagingVectorIO.push_back({const_cast<char *>(sEventSerialization.c_str()),sEventSerialization.size()});
        }

      const auto & controlMessageIO = controlMessageSerializer.getVectorIO();

      stagingVectorIO.insert(stagingVectorIO.end(),controlMessageIO.begin(),controlMessageIO.end());

      const auto & packetIO = pkt.getVectorIO();

      stagingVectorIO.insert(stagingVectorIO.end(),packetIO.begin(),packetIO.end());

      ++u64SequenceNumber_;

      size_t sentBytes{};
      PartInfo partInfo{false,0,0};

      while(sentBytes != totalSizeBytes)
        {
          EMANEMessage::OTAHeader otaheader;
          otaheader.set_source(pktInfo.getSource());
          otaheader.set_destination(pktInfo.getDestination());
          otaheader.set_sequence(u64SequenceNumber_);
          otaheader.set_uuid(reinterpret_cast<const char *>(uuid_),sizeof(uuid_));

          if(sentBytes==0)
            {
              auto pPayloadInfo = otaheader.mutable_payloadinfo();
              pPayloadInfo->set_datalength(pkt.length());
              pPayloadInfo->set_controllength(controlMessageSerializer.getLength());
              pPayloadInfo->set_eventlength(sEventSerialization.size());
            }

          std::string sOTAHeader{};

          if(!otaheader.SerializeToString(&sOTAHeader))
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "OTAManager sendOTAPacket unable to serialize OTA header src:%hu dst:%hu",
                                      pktInfo.getSource(),
                                      pktInfo.getDestination());
              break;
            }

          // total wire size includes 16 bit length prefix framing of header
          size_t totalWireSize = totalSizeBytes - sentBytes + (sOTAHeader.size() + 2) + sizeof(PartInfo);

          std::uint16_t u16HeaderLength = HTONS(sOTAHeader.size());

          Utils::VectorIO vectorIO{{reinterpret_cast<char *>(&u16HeaderLength),sizeof(u16HeaderLength)},
              {const_cast<char *>(sOTAHeader.c_str()),sOTAHeader.size()},
                {reinterpret_cast<char *>(&partInfo),sizeof(partInfo)}};

          size_t payloadSize{};

          if(otaMTU_ != 0 and totalWireSize > otaMTU_)
            {
              partInfo.u8More_ = 1;
              // size of payload only (event + control + packet data)
              // adjusted for MTU and overhead (OTAHeader +
              // PartInfo)
              payloadSize = otaMTU_ - (sOTAHeader.size() + 2 + sizeof(partInfo));
              partInfo.u32Size_ = HTONL(payloadSize);
            }
          else
            {
              partInfo.u8More_ = 0;
              // size of payload only (event + control + packet data)
              payloadSize = totalSizeBytes - sentBytes;
              partInfo.u32Size_ = HTONL(payloadSize);
            }

          partInfo.u32Offset_ = HTONL(totalSizeBytes - (totalSizeBytes - sentBytes));

          sentBytes += payloadSize;

          while(payloadSize)
            {
              size_t avaiableInEntrySize = stagingVectorIO[stagingIndex].iov_len - stagingOffset;

              if(avaiableInEntrySize > payloadSize)
                {
                  vectorIO.push_back({reinterpret_cast<char *>(stagingVectorIO[stagingIndex].iov_base) + stagingOffset,
                        payloadSize});

                  stagingOffset += payloadSize;
                  payloadSize = 0;
                }
              else
                {
                  vectorIO.push_back({reinterpret_cast<char *>(stagingVectorIO[stagingIndex].iov_base) + stagingOffset,
                        avaiableInEntrySize});

                  payloadSize -= avaiableInEntrySize;
                  stagingOffset = 0;
                  ++stagingIndex;
                }
            }

          // gather and send
          if(mcast_.send(&vectorIO[0],static_cast<int>(vectorIO.size())) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "OTAManager sendOTAPacket unable to send ctrl_len:%zu,"
                                      " payload_len:%zu src:%hu dst:%hu reason:%s\n",
                                      controlMessageSerializer.getLength(),
                                      pkt.length(),
                                      pktInfo.getSource(),
                                      pktInfo.getDestination(),
                                      strerror(errno));

            }
          else
            {
              otaStatisticPublisher_.update(OTAStatisticPublisher::Type::TYPE_DOWNSTREAM_PACKET_SUCCESS,
                                            uuid_,
                                            pktInfo.getSource());


              for(const auto & entry : eventSerializations)
                {
                  eventStatisticPublisher_.update(EventStatisticPublisher::Type::TYPE_TX,
                                                  uuid_,
                                                  std::get<1>(entry));
                }
            }
        }
    }

  // clean up control messages
  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::OTAManager::registerOTAUser(NEMId id, OTAUser * pOTAUser)
{
  std::pair<NEMUserMap::iterator, bool> ret;

  if(nemUserMap_.insert(std::make_pair(id,pOTAUser)).second == false)
    {
      std::stringstream ssDescription;
      ssDescription<<"attempted to register duplicate user with id "<<id<<std::ends;
      throw OTAException(ssDescription.str());
    }
}

void EMANE::OTAManager::unregisterOTAUser(NEMId id)
{
  if(nemUserMap_.erase(id) == 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"attempted to unregister unknown user with id "<<id<<std::ends;
      throw OTAException(ssDescription.str());
    }
}

void EMANE::OTAManager::open(const INETAddr & otaGroupAddress,
                             const std::string & otaManagerDevice,
                             bool bLoopback,
                             int iTTL,
                             const uuid_t & uuid,
                             size_t otaMTU,
                             Seconds partCheckThreshold,
                             Seconds partTimeoutThreshold)
{
  otaGroupAddress_ = otaGroupAddress;
  otaMTU_ = otaMTU;
  partCheckThreshold_ = partCheckThreshold;
  partTimeoutThreshold_ = partTimeoutThreshold;
  uuid_copy(uuid_,uuid);

  try
    {
      mcast_.open(otaGroupAddress,true,otaManagerDevice,iTTL,bLoopback);
    }
  catch(SocketException & exp)
    {
      std::stringstream sstream;

      sstream<<"Platform OTA Manager: Unable to open OTA Manager socket: '"
             <<otaGroupAddress.str()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * No Multicast device specified and routing table nondeterministic"
             <<std::endl
             <<"   (no multicast route and no default route)."
             <<std::endl
             <<" * Multicast device "
             <<otaManagerDevice
             <<" does not exist or is not up."
             <<std::endl
             <<exp.what()
             <<std::ends;

      throw OTAException(sstream.str());
    }

  thread_ = std::thread{&EMANE::OTAManager::processOTAMessage,this};

  if(ThreadUtils::elevate(thread_))
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,"OTAManager::open: Unable to set Real Time Priority");
    }

  bOpen_ = true;
}


void EMANE::OTAManager::processOTAMessage()
{
  unsigned char buf[65536];

  ssize_t len = 0;

  while(1)
    {
      if((len = mcast_.recv(buf,sizeof(buf),0)) > 0)
        {
          auto now =  Clock::now();

          // ota message len sanity check
          if(static_cast<size_t>(len) >= sizeof(std::uint16_t))
            {
              std::uint16_t * pu16OTAHeaderLength{reinterpret_cast<std::uint16_t *>(buf)};

              *pu16OTAHeaderLength = NTOHS(*pu16OTAHeaderLength);

              len -= sizeof(std::uint16_t);

              EMANEMessage::OTAHeader otaHeader;

              size_t payloadIndex{2 + *pu16OTAHeaderLength + sizeof(PartInfo)};

              if(static_cast<size_t>(len) >= *pu16OTAHeaderLength + sizeof(PartInfo) &&
                 otaHeader.ParseFromArray(&buf[2], *pu16OTAHeaderLength))
                {
                  PartInfo * pPartInfo{reinterpret_cast<PartInfo *>(&buf[2+*pu16OTAHeaderLength])};
                  pPartInfo->u32Offset_ = NTOHL(pPartInfo->u32Offset_);
                  pPartInfo->u32Size_ = NTOHL(pPartInfo->u32Size_);

                  uuid_t remoteUUID;
                  uuid_copy(remoteUUID,reinterpret_cast<const unsigned char *>(otaHeader.uuid().data()));

                  // only process messages that were not sent by this instance
                  if(uuid_compare(uuid_,remoteUUID))
                    {
                      // verify we have the advertized part length
                      if(static_cast<size_t>(len) ==
                         *pu16OTAHeaderLength +
                         sizeof(PartInfo) +
                         pPartInfo->u32Size_)
                        {
                          // message contained in a single part
                          if(!pPartInfo->u8More_  && !pPartInfo->u32Offset_)
                            {
                              auto & payloadInfo = otaHeader.payloadinfo();
                              handleOTAMessage(otaHeader.source(),
                                               otaHeader.destination(),
                                               remoteUUID,
                                               now,
                                               payloadInfo.eventlength(),
                                               payloadInfo.controllength(),
                                               payloadInfo.datalength(),
                                               {{&buf[payloadIndex],pPartInfo->u32Size_}});
                            }
                          else
                            {
                              PartKey partKey = PartKey{otaHeader.source(),otaHeader.sequence()};

                              auto iter = partStore_.find(partKey);

                              if(iter != partStore_.end())
                                {
                                  size_t & totalReceivedPartsBytes{std::get<0>(iter->second)};
                                  size_t & totalEventBytes{std::get<1>(iter->second)};
                                  size_t & totalControlBytes{std::get<2>(iter->second)};
                                  size_t & totalDataBytes{std::get<3>(iter->second)};
                                  auto & parts = std::get<4>(iter->second);
                                  auto & lastPartTime = std::get<5>(iter->second);

                                  // check to see if first part has been received
                                  if(otaHeader.has_payloadinfo())
                                    {
                                      auto & payloadInfo = otaHeader.payloadinfo();
                                      totalEventBytes = payloadInfo.eventlength();
                                      totalControlBytes = payloadInfo.controllength();
                                      totalDataBytes = payloadInfo.datalength();
                                    }

                                  // update last part receive time
                                  lastPartTime = now;

                                  // add this part to parts and update receive count
                                  totalReceivedPartsBytes +=  pPartInfo->u32Size_;

                                  parts.insert(std::make_pair(static_cast<size_t>(pPartInfo->u32Offset_),
                                                              std::vector<uint8_t>(&buf[payloadIndex],
                                                                                   &buf[payloadIndex + pPartInfo->u32Size_])));

                                  // determine if all parts are accounted for
                                  size_t totalExpectedPartsBytes = totalDataBytes + totalEventBytes + totalControlBytes;

                                  if(totalReceivedPartsBytes  == totalExpectedPartsBytes)
                                    {
                                      Utils::VectorIO vectorIO{};

                                      // get the parts sorted by offset and build an iovec
                                      for(const auto & part : parts)
                                        {
                                          vectorIO.push_back({const_cast<uint8_t *>(part.second.data()),
                                                part.second.size()});
                                        }

                                      handleOTAMessage(otaHeader.source(),
                                                       otaHeader.destination(),
                                                       remoteUUID,
                                                       now,
                                                       totalEventBytes,
                                                       totalControlBytes,
                                                       totalDataBytes,
                                                       vectorIO);

                                      // remove part cache and part time store
                                      partStore_.erase(iter);
                                    }
                                }
                              else
                                {
                                  PartKey partKey = PartKey{otaHeader.source(),otaHeader.sequence()};

                                  Parts parts{};

                                  parts.insert(std::make_pair(static_cast<size_t>(pPartInfo->u32Offset_),
                                                              std::vector<uint8_t>(&buf[payloadIndex],
                                                                                   &buf[payloadIndex + pPartInfo->u32Size_])));

                                  std::array<uint8_t,sizeof(uuid_t)> uuid;
                                  uuid_copy(uuid.data(),remoteUUID);

                                  // first part of message
                                  // check to see if first part has been received
                                  if(otaHeader.has_payloadinfo())
                                    {
                                      auto & payloadInfo = otaHeader.payloadinfo();

                                      partStore_.insert({partKey,
                                            std::make_tuple(static_cast<size_t>(pPartInfo->u32Size_),
                                                            payloadInfo.eventlength(),
                                                            payloadInfo.controllength(),
                                                            payloadInfo.datalength(),
                                                            parts,
                                                            now,
                                                            uuid)});
                                    }
                                  else
                                    {
                                      partStore_.insert({partKey,
                                            std::make_tuple(static_cast<size_t>(pPartInfo->u32Size_),
                                                            0, // event length
                                                            0, // control length
                                                            0, // data length
                                                            parts,
                                                            now,
                                                            uuid)});
                                    }
                                }
                            }
                        }
                      else
                        {
                          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                  ERROR_LEVEL,
                                                  "OTAManager message part size mismatch");
                        }
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "OTAManager message header could not be deserialized");
                }
            }
          else
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "OTAManager message missing header missing prefix length encoding");
            }

          // check to see if there are part assemblies to abandon
          if(lastPartCheckTime_ + partCheckThreshold_ <= now)
            {
              for(auto iter = partStore_.begin(); iter != partStore_.end();)
                {
                  auto & lastPartTime = std::get<5>(iter->second);

                  if(lastPartTime + partTimeoutThreshold_ <= now)
                    {
                      auto & srcNEM = std::get<0>(iter->first);
                      uuid_t uuid;
                      uuid_copy(uuid,std::get<6>(iter->second).data());

                      otaStatisticPublisher_.update(OTAStatisticPublisher::Type::TYPE_UPSTREAM_PACKET_DROP_MISSING_PARTS,
                                                    uuid,
                                                    srcNEM);

                      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                              ERROR_LEVEL,
                                              "OTAManager missing one or more packet parts src:"
                                              " %hu sequence: %ju, dropping.",
                                              srcNEM,
                                              std::get<1>(iter->first));

                      partStore_.erase(iter++);
                    }
                  else
                    {
                      ++iter;
                    }
                }

              lastPartCheckTime_ = now;
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "OTAManager Packet Received error");
          break;
        }

    }
}


void  EMANE::OTAManager::handleOTAMessage(NEMId source,
                                          NEMId destination,
                                          const uuid_t & remoteUUID,
                                          const TimePoint & now,
                                          size_t eventsSize,
                                          size_t controlsSize,
                                          size_t dataSize,
                                          const Utils::VectorIO & vectorIO)
{
  size_t index{};
  size_t offset{};

  if(eventsSize)
    {
      std::vector<uint8_t> buf{bufferFromVectorIO(eventsSize,
                                                  index,
                                                  offset,
                                                  vectorIO)};
      EMANEMessage::Event::Data data;

      if(data.ParseFromArray(&buf[0],eventsSize))
        {
          for(const auto & serialization : data.serializations())
            {
              EventServiceSingleton::instance()->processEventMessage(serialization.nemid(),
                                                                     serialization.eventid(),
                                                                     serialization.data());

              eventStatisticPublisher_.update(EventStatisticPublisher::Type::TYPE_RX,
                                              remoteUUID,
                                              serialization.eventid());
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "OTAManager message events could not be deserialized");
        }
    }

  Controls::OTATransmitters otaTransmitters{};

  if(controlsSize)
    {
      std::vector<uint8_t> buf{bufferFromVectorIO(controlsSize,
                                                  index,
                                                  offset,
                                                  vectorIO)};

      ControlMessages msgs =
        ControlMessageSerializer::create(&buf[0],
                                         controlsSize);

      for(ControlMessages::const_iterator iter = msgs.begin(),end = msgs.end();
          iter != end;
          ++iter)
        {
          if((*iter)->getId() == Controls::SerializedControlMessage::IDENTIFIER)
            {
              auto pSerializedControlMessage =
                static_cast<const Controls::SerializedControlMessage *>(*iter);

              if(pSerializedControlMessage->getSerializedId() ==
                 Controls::OTATransmitterControlMessage::IDENTIFIER)
                {
                  std::unique_ptr<Controls::OTATransmitterControlMessage>
                    pOTATransmitterControlMessage(Controls::OTATransmitterControlMessage::
                                                  create(pSerializedControlMessage->getSerialization()));

                  otaTransmitters = pOTATransmitterControlMessage->getOTATransmitters();
                }

            }

          // delete all control messages
          delete *iter;
        }
    }

  // create packet info from the ota data message
  PacketInfo pktInfo(source,
                     destination,
                     0,
                     now,
                     remoteUUID);

  Utils::VectorIO packetVectorIO{};

  for(; index < vectorIO.size(); ++index, offset=0)
    {
      packetVectorIO.push_back({reinterpret_cast<uint8_t *>(vectorIO[index].iov_base) + offset,
            vectorIO[index].iov_len - offset});
    }

  UpstreamPacket pkt(pktInfo,packetVectorIO);

  if(pkt.length() == dataSize)
    {
      otaStatisticPublisher_.update(OTAStatisticPublisher::Type::TYPE_UPSTREAM_PACKET_SUCCESS,
                                    remoteUUID,
                                    pktInfo.getSource());

      // for each local NEM stack
      for(NEMUserMap::const_iterator iter = nemUserMap_.begin(), end = nemUserMap_.end();
          iter != end; ++iter)
        {
          // only send pkt up to NEM(s) NOT in the ATS
          if(otaTransmitters.count(iter->first) == 0)
            {
              iter->second->processOTAPacket(pkt,ControlMessages());
            }
        }
    }
  else
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,
                              "OTAManager packet size does not match reported size in OTA header");
    }
}
