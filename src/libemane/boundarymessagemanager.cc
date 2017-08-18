/*
 * Copyright (c) 2013-2017 - Adjacent Link LLC, Bridgewater, New Jersey
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
#include <sys/socket.h>
#include <unistd.h>

EMANE::BoundaryMessageManager::BoundaryMessageManager(NEMId id):
  id_{id},
  bOpen_{},
  iSockFd_{-1},
  iSessionSockFd_{-1},
  bConnected_{}{}


EMANE::BoundaryMessageManager::~BoundaryMessageManager()
{
  if(thread_.joinable())
    {
      close();
    }
}

void EMANE::BoundaryMessageManager::open(const INETAddr & localAddress,
                                         const INETAddr & remoteAddress,
                                         Protocol protocol)
{
  localAddress_   = localAddress;
  remoteAddress_  = remoteAddress;
  protocol_ = protocol;

  try
    {
      if(protocol_ == Protocol::PROTOCOL_UDP)
        {
          udp_.open(localAddress_,true);
        }
      // note: Protocol::PROTOCOL_TCP_* sockets open in thread
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

  if(protocol_ == Protocol::PROTOCOL_UDP)
    {
      thread_ = std::thread{&BoundaryMessageManager::processNetworkMessageUDP,this};
    }
  else
    {
      thread_ = std::thread{&BoundaryMessageManager::processNetworkMessageTCP,this};
    }

  bOpen_ = true;
}

void EMANE::BoundaryMessageManager::close()
{
  if(bOpen_)
    {
      if(thread_.joinable())
        {
          ThreadUtils::cancel(thread_);

          thread_.join();
        }

      if(protocol_ == Protocol::PROTOCOL_UDP)
        {
          udp_.close();
        }
      else
        {
          ::close(iSockFd_);

          if(bConnected_ &&  protocol_ == Protocol::PROTOCOL_TCP_SERVER)
            {
              ::close(iSessionSockFd_);
            }
        }


      bOpen_ = false;
    }
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
  dataMessage.u32DataLen_ = packetDataLength;
  dataMessage.u32CtrlLen_ = controlMessageSerializer.getLength();

  NetAdapterDataMessageToNet(&dataMessage);

  NetAdapterHeader header;
  memset(&header,0,sizeof(header));
  header.u16Id_ = NETADAPTER_DATA_MSG;
  header.u32Length_ =
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

  if(protocol_ == Protocol::PROTOCOL_UDP)
    {
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
    }
  else
    {
      msghdr msg;
      memset(&msg,0,sizeof(msg));
      msg.msg_iov = const_cast<iovec *>(&vectorIO[0]);
      msg.msg_iovlen = vectorIO.size();

      std::lock_guard<std::mutex> m(mutex_);

      if(bConnected_)
        {
          if((len = sendmsg(iSessionSockFd_,&msg,0)) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,"NEM %03d BoundaryMessageManager error "
                                      "on message send (expected:%zu actual:%zd)",
                                      id_,
                                      packetDataLength,
                                      len);
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,"NEM %03d BoundaryMessageManager "
                                  "not connected",
                                  id_);
        }
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
  header.u32Length_ = sizeof(header) + controlMessageSerializer.getLength() + 4;

  NetAdapterHeaderToNet(&header);

  NetAdapterControlMessage controlMessage;

  memset(&controlMessage,0,sizeof(NetAdapterControlMessage));
  controlMessage.u32CtrlLen_ = controlMessageSerializer.getLength();

  NetAdapterControlMessageToNet(&controlMessage);

  Utils::VectorIO vectorIO;
  vectorIO.push_back({&header,sizeof(header)});
  vectorIO.push_back({&controlMessage,sizeof(controlMessage)});

  const Utils::VectorIO & controlMessageVectorIO =
    controlMessageSerializer.getVectorIO();

  vectorIO.insert(vectorIO.end(),
                  controlMessageVectorIO.begin(),
                  controlMessageVectorIO.end());

  if(protocol_ == Protocol::PROTOCOL_UDP)
    {
      if((len = udp_.send(&vectorIO[0],
                          static_cast<int>(vectorIO.size()),
                          remoteAddress_)) == -1)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,"NEM %03d BoundaryMessageManager "
                                  "error on control message send (expected:%u actual:%zu)",
                                  id_,
                                  header.u32Length_,
                                  len);
        }
    }
  else
    {
      msghdr msg;
      memset(&msg,0,sizeof(msg));

      msg.msg_iov = const_cast<iovec *>(&vectorIO[0]);
      msg.msg_iovlen = vectorIO.size();

      std::lock_guard<std::mutex> m(mutex_);

      if(bConnected_)
        {
          if((len =  sendmsg(iSessionSockFd_,&msg,0)) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,"NEM %03d BoundaryMessageManager "
                                      "error on control message send (expected:%u actual:%zu)",
                                      id_,
                                      header.u32Length_,
                                      len);
            }
        }
      else
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  DEBUG_LEVEL,"NEM %03d BoundaryMessageManager "
                                  "not connected",
                                  id_);
        }
    }

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}


void EMANE::BoundaryMessageManager::handleNetworkMessage(void * buf,size_t len)
{
  NEMId dstNemId{};

  if(static_cast<size_t>(len) >= sizeof(NetAdapterHeader))
    {
      NetAdapterHeader * pHeader = reinterpret_cast<NetAdapterHeader *>(buf);

      NetAdapterHeaderToHost(pHeader);

      if(static_cast<size_t>(len) == pHeader->u32Length_)
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

                  if(static_cast<size_t>(len) >= pMsg->u32DataLen_)
                    {
                      PacketInfo pinfo{pMsg->u16Src_, dstNemId, pMsg->u8Priority_,Clock::now()};


                      UpstreamPacket pkt(pinfo,
                                         pMsg->data_,
                                         pMsg->u32DataLen_);

                      len -= pMsg->u32DataLen_;

                      ControlMessages controlMessages;

                      if(static_cast<size_t>(len) ==  pMsg->u32CtrlLen_)
                        {
                          const ControlMessages & controlMessages =
                            ControlMessageSerializer::create(pMsg->data_ + pMsg->u32DataLen_,
                                                             pMsg->u32CtrlLen_);

                          try
                            {
                              doProcessPacketMessage(pinfo,
                                                     pMsg->data_,
                                                     pMsg->u32DataLen_,
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

                  if(static_cast<size_t>(len) == pMsg->u32CtrlLen_)
                    {
                      const ControlMessages & controlMessages =
                        ControlMessageSerializer::create(pMsg->data_,
                                                         pMsg->u32CtrlLen_);

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
                                  pHeader->u32Length_,
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

void EMANE::BoundaryMessageManager::processNetworkMessageUDP()
{
  unsigned char buf[65536];
  ssize_t len = 0;

  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          DEBUG_LEVEL,
                          "NEM %03d BoundaryMessageManager::processNetworkMessage",
                          id_);

  while(1)
    {
      if((len = udp_.recv(buf,sizeof(buf),0)) > 0)
        {
          LOGGER_VERBOSE_LOGGING(*LogServiceSingleton::instance(),
                                 DEBUG_LEVEL,
                                 "NEM %03d BoundaryMessageManager Pkt Rcvd len: %zd",
                                 id_,len);

          handleNetworkMessage(buf,len);
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

void EMANE::BoundaryMessageManager::processNetworkMessageTCP()
{
  unsigned char buf[1024];
  std::uint32_t u32MessageSizeBytes{};
  std::vector<uint8_t> message{};
  bool bLocalConnected{};
  bool bLocalOpen{};
  int iSockFd{-1};

  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                          DEBUG_LEVEL,
                          "NEM %03d BoundaryMessageManager::processNetworkMessageTCP",
                          id_);
  while(1)
    {
      if(!bLocalOpen)
        {
          if((iSockFd = socket(localAddress_.getFamily(),
                               SOCK_STREAM,
                               0)) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEM %03d BoundaryMessageManager socket open error: %s",
                                      id_,
                                      strerror(errno));

              break;
            }

          int iOption{1};

          if(setsockopt(iSockFd,
                        SOL_SOCKET,
                        SO_REUSEADDR,
                        reinterpret_cast<void*>(&iOption),
                        sizeof(iOption)) < 0)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEM %03d BoundaryMessageManager socket SO_REUSEADDR error: %s",
                                      id_,
                                      strerror(errno));

              break;
            }

          if(::bind(iSockFd,
                    localAddress_.getSockAddr(),
                    localAddress_.getAddrLength()) < 0)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEM %03d BoundaryMessageManager socket bind error: %s",
                                      id_,
                                      strerror(errno));
              break;
            }

          if(protocol_ == Protocol::PROTOCOL_TCP_SERVER)
            {
              if(listen(iSockFd,1) < 0)
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "NEM %03d BoundaryMessageManager socket listen error: %s",
                                          id_,
                                          strerror(errno));
                  break;
                }
            }

          bLocalOpen = true;

          // store socket for proper shutdown on boundry close
          std::lock_guard<std::mutex> m(mutex_);
          iSockFd_ = iSockFd;
        }

      int iConnectionSockFd{};

      while(!bLocalConnected)
        {
          if(protocol_ == Protocol::PROTOCOL_TCP_SERVER)
            {
              if((iConnectionSockFd = accept(iSockFd_,nullptr,nullptr)) > 0)
                {
                  bLocalConnected = true;
                }
              else
                {
                  if(errno == EINTR)
                    {
                      continue;
                    }
                  else
                    {
                      return;
                    }
                }
            }
          else if(protocol_ == Protocol::PROTOCOL_TCP_CLIENT)
            {
              if(connect(iSockFd_,
                         remoteAddress_.getSockAddr(),
                         remoteAddress_.getAddrLength()))
                {
                  // wait and try again
                  sleep(1);
                }
              else
                {
                  bLocalConnected = true;
                }
            }
        }

      // allow sending methods to transmit
      mutex_.lock();

      bConnected_ = bLocalConnected;

      if(protocol_ == Protocol::PROTOCOL_TCP_SERVER)
        {
          iSessionSockFd_ = iConnectionSockFd;
        }
      else if(protocol_ == Protocol::PROTOCOL_TCP_CLIENT)
        {
          iSessionSockFd_ = iSockFd_;
          iConnectionSockFd = iSockFd_;
        }

      mutex_.unlock();

      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              INFO_LEVEL,
                              "NEM %03d BoundaryMessageManager connected",
                              id_);
      while(bLocalConnected)
        {
          bool bDoDisconnect{};

          // process the session data
          if(u32MessageSizeBytes == 0)
            {
              ssize_t length{}; // number of bytes received

              // read at most 6 bytes from peer to determine message length
              length = recv(iConnectionSockFd,buf,6 - message.size(),0);

              if(length > 0)
                {
                  // save the contents of the message header
                  message.insert(message.end(),&buf[0],&buf[length]);

                  // is the entire header present
                  if(message.size() == 6)
                    {
                      u32MessageSizeBytes = ntohl(*reinterpret_cast<std::uint32_t *>(&message[2]));

                      // a message length of 0 is not allowed
                      if(!u32MessageSizeBytes)
                        {
                          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                  ERROR_LEVEL,
                                                  "NEM %03d BoundaryMessageManager Message Header read error",
                                                  id_);
                          bDoDisconnect = true;
                        }
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "NEM %03d BoundaryMessageManager Message Header read error",
                                          id_);
                  bDoDisconnect = true;
                }
            }
          else
            {
              // attempt to read message length remaining or max buffer size
              ssize_t length{};

              length = recv(iConnectionSockFd,
                            buf,
                            u32MessageSizeBytes - message.size() > sizeof(buf) ?
                            sizeof(buf) : u32MessageSizeBytes - message.size(),
                            0);

              if(length > 0)
                {
                  message.insert(message.end(),&buf[0],&buf[length]);

                  // process message when full message is read
                  if(message.size() == u32MessageSizeBytes)
                    {
                      LOGGER_VERBOSE_LOGGING(*LogServiceSingleton::instance(),
                                             DEBUG_LEVEL,
                                             "NEM %03d BoundaryMessageManager Pkt Rcvd len: %zd",
                                             id_,
                                             message.size());

                      handleNetworkMessage(&message[0],message.size());

                      message.clear();
                      u32MessageSizeBytes = 0;
                    }
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                          ERROR_LEVEL,
                                          "NEM %03d BoundaryMessageManager Message Header read error",
                                          id_);

                  bDoDisconnect = true;
                }
            }

          if(bDoDisconnect)
            {
              message.clear();
              u32MessageSizeBytes = 0;
              bLocalConnected = false;
            }
        }

      std::lock_guard<std::mutex> m(mutex_);
      bConnected_ = false;

      // iConnectionSockFd == iSocketFd for TCP_CLIENT
      ::close(iConnectionSockFd);

      if(protocol_ == Protocol::PROTOCOL_TCP_CLIENT)
        {
          bLocalOpen = false;
        }

      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              INFO_LEVEL,
                              "NEM %03d BoundaryMessageManager disconnected",
                              id_);

    }
}
