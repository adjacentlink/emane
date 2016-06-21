/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
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

EMANE::OTAManager::OTAManager():
  bOpen_(false),
  eventStatisticPublisher_{"OTAChannel"},
  u64SequenceNumber_{}
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
                                      DownstreamPacket & pkt,
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

      otaheader.set_source(pktInfo.getSource());
      otaheader.set_destination(pktInfo.getDestination());
      otaheader.set_datalength(pkt.length());
      otaheader.set_controllength(controlMessageSerializer.getLength());
      otaheader.set_eventlength(sEventSerialization.size());
      otaheader.set_sequencenumber(++u64SequenceNumber_);
      otaheader.set_uuid(reinterpret_cast<const char *>(uuid_),sizeof(uuid_));

      std::string sOTAHeader{};

      if(otaheader.SerializeToString(&sOTAHeader))
        {
          std::uint16_t u16HeaderLength = HTONS(sOTAHeader.size());

          Utils::VectorIO vectorIO{{reinterpret_cast<char *>(&u16HeaderLength),sizeof(u16HeaderLength)},
              {const_cast<char *>(sOTAHeader.c_str()),sOTAHeader.size()}};

          if(!sEventSerialization.empty())
            {
              vectorIO.push_back({const_cast<char *>(sEventSerialization.c_str()),sEventSerialization.size()});
            }

          const auto  & controlMessageIO = controlMessageSerializer.getVectorIO();

          vectorIO.insert(vectorIO.end(),controlMessageIO.begin(),controlMessageIO.end());

          const auto & packetIO = pkt.getVectorIO();

          vectorIO.insert(vectorIO.end(),packetIO.begin(),packetIO.end());

          // gather and send
          if(mcast_.send(&vectorIO[0],static_cast<int>(vectorIO.size())) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "OTAManager sendOTAPacket unable to send ctrl_len:%zu, payload_len:%zu src:%hu dst:%hu reason:%s\n",
                                      controlMessageSerializer.getLength(),
                                      pkt.length(),
                                      pktInfo.getSource(),
                                      pktInfo.getDestination(),
                                      strerror(errno));

            }
          else
            {
              otaStatisticPublisher_.update(OTAStatisticPublisher::Type::TYPE_DOWNSTREAM,
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
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "OTAManager sendOTAPacket unable to serialize OTA header src:%hu dst:%hu",
                                  pktInfo.getSource(),
                                  pktInfo.getDestination());
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
                             const uuid_t & uuid)
{
  otaGroupAddress_ = otaGroupAddress;

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

              if(static_cast<size_t>(len) >= *pu16OTAHeaderLength &&
                 otaHeader.ParseFromArray(&buf[2], *pu16OTAHeaderLength))
                {
                  if(static_cast<size_t>(len) ==
                     otaHeader.datalength() +
                     otaHeader.controllength() +
                     otaHeader.eventlength() +
                     *pu16OTAHeaderLength)
                    {
                      std::uint16_t u16EventIndex = 2 + *pu16OTAHeaderLength;
                      std::uint16_t u16ControlIndex = u16EventIndex + otaHeader.eventlength();
                      std::uint16_t u16PacketIndex = u16ControlIndex + otaHeader.controllength();

                      uuid_t remoteUUID;
                      uuid_copy(remoteUUID,reinterpret_cast<const unsigned char *>(otaHeader.uuid().data()));

                      if(uuid_compare(uuid_,remoteUUID))
                        {
                          if(otaHeader.eventlength())
                            {
                              EMANEMessage::Event::Data data;

                              if(data.ParseFromArray(&buf[u16EventIndex],otaHeader.eventlength()))
                                {
                                  using RepeatedPtrFieldSerilaization =
                                    google::protobuf::RepeatedPtrField<EMANEMessage::Event::Data::Serialization>;

                                  for(const auto & repeatedSerialization :
                                        RepeatedPtrFieldSerilaization(data.serializations()))
                                    {
                                      EventServiceSingleton::instance()->processEventMessage(repeatedSerialization.nemid(),
                                                                                             repeatedSerialization.eventid(),
                                                                                             repeatedSerialization.data());

                                      eventStatisticPublisher_.update(EventStatisticPublisher::Type::TYPE_RX,
                                                                      remoteUUID,
                                                                      repeatedSerialization.eventid());
                                    }
                                }
                              else
                                {
                                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                          ERROR_LEVEL,"OTAManager message events could not be deserialized");
                                }
                            }

                          // create packet info from the ota data message
                          PacketInfo pktInfo(otaHeader.source(),
                                             otaHeader.destination(),
                                             0,
                                             now,
                                             remoteUUID);

                          UpstreamPacket pkt(pktInfo,&buf[u16PacketIndex],otaHeader.datalength());

                          Controls::OTATransmitters otaTransmitters;

                          if(otaHeader.controllength())
                            {
                              ControlMessages msgs =
                                ControlMessageSerializer::create(&buf[u16ControlIndex],
                                                                 otaHeader.controllength());

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

                          otaStatisticPublisher_.update(OTAStatisticPublisher::Type::TYPE_UPSTREAM,
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
                    }
                  else
                    {
                      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                              ERROR_LEVEL,
                                              "OTAManager Packet received data length incorrect"
                                              " len: %zd header:%hu data:%u control: %u event: %u ",
                                              len,
                                              *pu16OTAHeaderLength,
                                              otaHeader.datalength(),
                                              otaHeader.controllength(),
                                              otaHeader.eventlength());
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,"OTAManager message header could not be deserialized");
                }
            }
          else
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,"OTAManager message missing header missing prefix length encoding");
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,"OTAManager Packet Received error");
          break;
        }
    }
}
