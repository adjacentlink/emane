/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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
 * * Neither the name of Adjacent Link LLC nor the names of its
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

#include "boundarymessagemanager.h"
#include "boundarymessagemanagerexception.h"
#include "logservice.h"
#include "controlmessageserializer.h"
#include "netadaptermessage.h"

#include "emane/utils/threadutils.h"
#include "emane/controls/serializedcontrolmessage.h"
#include "emane/upstreampacket.h"

#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstring>

EMANE::BoundaryMessageManager::BoundaryMessageManager(NEMId id):
  id_{id},
  bOpen_{}{}

EMANE::BoundaryMessageManager::~BoundaryMessageManager()
{
  if(thread_.joinable())
     {
       close();
     }
}

void EMANE::BoundaryMessageManager::open(const INETAddr & localAddress,
                                         const INETAddr & remoteAddress)
{
  localAddress_   = localAddress;
  remoteAddress_  = remoteAddress;

  try
    {
      udp_.open(localAddress_,true);
    }
  catch(...)
    {
      std::stringstream sstream;

      sstream<<"NEM "
             <<std::setw(3)
             <<std::setfill('0')
             <<id_
             <<": Unable to open receive socket to transport: '"
             <<localAddress_.str()
             <<"'."
             <<std::endl
             <<std::endl
             <<"Possible reason(s):"
             <<std::endl
             <<" * "
             <<localAddress_.str()
             <<" is not this host."
             <<std::endl
             <<std::ends;

      throw BoundaryMessageManagerException(sstream.str());
    }

  thread_ = std::thread{&BoundaryMessageManager::processNetworkMessage,this};

  bOpen_ = true;
}

void EMANE::BoundaryMessageManager::close()
{
  if(thread_.joinable())
    {
      ThreadUtils::cancel(thread_);

      thread_.join();
    }

  udp_.close();

  bOpen_ = false;
}

void EMANE::BoundaryMessageManager::sendPacketMessage(const PacketInfo & packetInfo,
                                                      const void * pPacketData,
                                                      size_t packetDataLength,
                                                      const ControlMessages & msgs)
{
  return sendPacketMessage(packetInfo,
                           Utils::VectorIO{{const_cast<void *>(pPacketData),packetDataLength}},
                           packetDataLength,
                           msgs);
}


void EMANE::BoundaryMessageManager::sendPacketMessage(const PacketInfo & packetInfo,
                                                      const Utils::VectorIO & packetIO,
                                                      size_t packetDataLength,
                                                      const ControlMessages & msgs)
{
  ssize_t len = 0;

  ControlMessageSerializer controlMessageSerializer(msgs);

  NetAdapterDataMessage dataMessage;
  memset(&dataMessage,0,sizeof(dataMessage));

  dataMessage.u16Src_ = packetInfo.getSource();
  dataMessage.u16Dst_ = packetInfo.getDestination();
  dataMessage.u8Priority_ = packetInfo.getPriority();
  dataMessage.u16DataLen_ = packetDataLength;
  dataMessage.u16CtrlLen_ = controlMessageSerializer.getLength();

  NetAdapterDataMessageToNet(&dataMessage);

  NetAdapterHeader header;
  memset(&header,0,sizeof(header));
  header.u16Id_ = NETADAPTER_DATA_MSG;
  header.u16Length_ =
    sizeof(header) +
    sizeof(dataMessage) +
    packetDataLength +
    controlMessageSerializer.getLength();

  NetAdapterHeaderToNet(&header);

  const Utils::VectorIO & controlVectorIO =
    controlMessageSerializer.getVectorIO();

  Utils::VectorIO vectorIO;
  vectorIO.push_back({&header,sizeof(header)});
  vectorIO.push_back({&dataMessage,sizeof(dataMessage)});

  vectorIO.insert(vectorIO.end(),
                  packetIO.begin(),
                  packetIO.end());

  vectorIO.insert(vectorIO.end(),
                  controlVectorIO.begin(),
                  controlVectorIO.end());

  if((len = udp_.send(&vectorIO[0],
                      static_cast<int>(vectorIO.size()),
                      remoteAddress_)) == -1)
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,"NEM %03d BoundaryMessageManager error "
                              "on message send (expected:%zu actual:%zd)",
                              id_,
                              packetDataLength,
                              len);
    }

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::BoundaryMessageManager::sendControlMessage(const ControlMessages & msgs)
{
  ssize_t len = 0;

  ControlMessageSerializer controlMessageSerializer(msgs);

  NetAdapterHeader header;
  memset(&header,0,sizeof(header));

  header.u16Id_ =  NETADAPTER_CTRL_MSG;
  header.u16Length_ = sizeof(header) + controlMessageSerializer.getLength() + 2;

  NetAdapterHeaderToNet(&header);

  NetAdapterControlMessage controlMessage;

  memset(&controlMessage,0,sizeof(NetAdapterControlMessage));
  controlMessage.u16CtrlLen_ = controlMessageSerializer.getLength();

  NetAdapterControlMessageToNet(&controlMessage);

  Utils::VectorIO vectorIO;
  vectorIO.push_back({&header,sizeof(header)});
  vectorIO.push_back({&controlMessage,sizeof(controlMessage)});

  const Utils::VectorIO & controlMessageVectorIO =
    controlMessageSerializer.getVectorIO();

  vectorIO.insert(vectorIO.end(),
                             controlMessageVectorIO.begin(),
                             controlMessageVectorIO.end());

  if((len = udp_.send(&vectorIO[0],
                      static_cast<int>(vectorIO.size()),
                      remoteAddress_)) == -1)
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,"NEM %03d BoundaryMessageManager "
                              "error on control message send (expected:%hu actual:%zu)",
                               id_,
                               header.u16Length_,
                               len);
    }

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::BoundaryMessageManager::processNetworkMessage()
{
  unsigned char buf[65536];
  ssize_t len = 0;
  NEMId dstNemId;

  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          DEBUG_LEVEL,
                          "NEM %03d BoundaryMessageManager::processNetworkMessage",
                          id_);

  while(1)
    {
      dstNemId = 0;

      memset(&buf,0,sizeof(buf));

      if((len = udp_.recv(buf,sizeof(buf),0)) > 0)
        {
          LOGGER_VERBOSE_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,
                                  "NEM %03d BoundaryMessageManager Pkt Rcvd len: %zd",
                                  id_,len);

          if(static_cast<size_t>(len) >= sizeof(NetAdapterHeader))
            {
              NetAdapterHeader * pHeader = reinterpret_cast<NetAdapterHeader *>(buf);

              NetAdapterHeaderToHost(pHeader);

              if(static_cast<size_t>(len) == pHeader->u16Length_)
                {
                  len -= sizeof(NetAdapterHeader);

                  switch(pHeader->u16Id_)
                    {
                    case NETADAPTER_DATA_MSG:

                      if(static_cast<size_t>(len) >= sizeof(NetAdapterDataMessage))
                        {
                          NetAdapterDataMessage * pMsg =  reinterpret_cast<NetAdapterDataMessage *>(pHeader->data_);

                          NetAdapterDataMessageToHost(pMsg);

                          len -= sizeof(NetAdapterDataMessage);

                          if(pMsg->u16Dst_ == htons(NETADAPTER_BROADCAST_ADDRESS))
                            {
                              dstNemId = NEM_BROADCAST_MAC_ADDRESS;
                            }
                          else
                            {
                              dstNemId = pMsg->u16Dst_;
                            }

                          if(static_cast<size_t>(len) >= pMsg->u16DataLen_)
                            {
                              PacketInfo pinfo{pMsg->u16Src_, dstNemId, pMsg->u8Priority_,Clock::now()};


                              UpstreamPacket pkt(pinfo,
                                                 pMsg->data_,
                                                 pMsg->u16DataLen_);

                              len -= pMsg->u16DataLen_;

                              ControlMessages controlMessages;

                              if(static_cast<size_t>(len) ==  pMsg->u16CtrlLen_)
                                {
                                  const ControlMessages & controlMessages =
                                    ControlMessageSerializer::create(pMsg->data_ + pMsg->u16DataLen_,
                                                                     pMsg->u16CtrlLen_);

                                  try
                                    {
                                      doProcessPacketMessage(pinfo,
                                                             pMsg->data_,
                                                             pMsg->u16DataLen_,
                                                             controlMessages);
                                    }
                                  catch(...)
                                    {
                                      // cannot really do too much at this point, so we'll log it
                                      // good candidate spot to generate and error event as well
                                      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                              ERROR_LEVEL,
                                                              "NEM %03d BoundaryMessageManager::processNetworkMessage "
                                                              "processUpstreamPacket exception caught",
                                                              id_);
                                    }
                                }
                              else
                                {
                                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                          ERROR_LEVEL,
                                                          "NEM %03d BoundaryMessageManager::processNetworkMessage "
                                                          "processUpstreamPacket size too small for control message data",
                                                          id_);


                                }

                            }
                          else
                            {
                              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                      ERROR_LEVEL,
                                                      "NEM %03d BoundaryMessageManager::processNetworkMessage "
                                                      "processUpstreamPacket size too small for data message",
                                                      id_);
                            }
                        }
                      else
                        {
                          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                  ERROR_LEVEL,
                                                  "NEM %03d BoundaryMessageManager::processNetworkMessage "
                                                  "processUpstreamPacket size too small for packet data message",
                                                  id_);
                        }

                      break;

                    case NETADAPTER_CTRL_MSG:

                      if(static_cast<size_t>(len)  >= sizeof(NetAdapterControlMessage))
                        {
                          NetAdapterControlMessage * pMsg =
                            reinterpret_cast<NetAdapterControlMessage *>(pHeader->data_);

                          NetAdapterControlMessageToHost(pMsg);

                          len -= sizeof(NetAdapterControlMessage);

                          ControlMessages controlMessages;

                          if(static_cast<size_t>(len) ==  pMsg->u16CtrlLen_)
                            {
                              const ControlMessages & controlMessages =
                                ControlMessageSerializer::create(pMsg->data_,
                                                                 pMsg->u16CtrlLen_);

                              try
                                {
                                  doProcessControlMessage(controlMessages);
                                }
                              catch(...)
                                {
                                  // cannot really do too much at this point, so we'll log it
                                  // good candidate spot to generate and error event as well
                                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                          ERROR_LEVEL,
                                                          "NEM %03d BoundaryMessageManager::processNetworkMessage "
                                                          "processDownstreamControl exception caught",
                                                          id_);
                                }
                            }
                          else
                            {
                              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                      ERROR_LEVEL,
                                                      "NEM %03d BoundaryMessageManager control message size mismatch",
                                                      id_);
                            }
                        }

                      break;

                    default:
                      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                              ERROR_LEVEL,
                                              "NEM %03d BoundaryMessageManager Received unknown message type: %d",
                                              id_,
                                              pHeader->u16Id_);
                      break;
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "NEM %03d BoundaryMessageManager Message mismatch expected %hd got %zd",
                                          id_,
                                          pHeader->u16Length_,
                                          len);
                }
            }
          else
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEM %03d BoundaryMessageManager Message Header mismatch expected %zu got %zd",
                                      id_,
                                      sizeof(NetAdapterHeader),
                                      len);
            }

        }
      else
        {
          // read error
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "NEM %03d BoundaryMessageManager Message Header read error",
                                  id_);

          break;
        }
    }
}
