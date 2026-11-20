/*
 * Copyright (c) 2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "transport.h"

#include "emane/downstreampacket.h"
#include "emane/upstreampacket.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"
#include "emane/utils/parameterconvert.h"

#include "traffic.pb.h"

#include <unistd.h>

EMANE::Transports::Traffic::
Transport::Transport(NEMId id,
                     PlatformServiceProvider * pPlatformService):
  EMANE::Transport(id, pPlatformService),
  iStreamSocketFd_{-1},
  session_{id,pPlatformService},
  u64SequenceNumber_{}
{}

EMANE::Transports::Traffic::Transport::~Transport()
{}

void EMANE::Transports::Traffic::Transport::initialize(Registrar & registrar)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s",
                          id_,
                          __func__);

  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.
    registerNonNumeric<INETAddr>("listenendpoint",
                                 ConfigurationProperties::REQUIRED,
                                 {},
                                 "Defines the TCP endpoint for receiving"
                                 " traffic messages with the following"
                                 " format:"
                                 " IPv4:PORT|[IPv6]:PORT.");

  configRegistrar.
    registerNonNumeric<std::string>("map",
                                    ConfigurationProperties::REQUIRED,
                                    {},
                                    "Defines mapping (source, destination, and"
                                    " local) to NEM id with the"
                                    " following format:"
                                    " <label>:<nem id>.",
                                    1,
                                    std::numeric_limits<std::uint16_t>::max(),
                                    "^.+:\\d+$");
}

void
EMANE::Transports::Traffic::
Transport::configure(const ConfigurationUpdate & update)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s",
                          id_,
                          __func__);

  for(const auto & item : update)
    {
      if(item.first == "listenendpoint")
        {
          listenEndpoint_ =  item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu"
                                  " Traffic::Transport::%s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  listenEndpoint_.str().c_str());
        }
      else if(item.first == "map")
        {
          for(const auto & entry: item.second)
            {
              const auto & sEntry = entry.asString();

              auto pos = sEntry.find_last_of(':');

              std::string sLabel = sEntry.substr(0,pos);

              NEMId nemId =
                Utils::ParameterConvert(sEntry.substr(pos+1)).toUINT16();

              if(!labelToNEMMap_.emplace(sLabel,nemId).second)
                {
                  throw ConfigureException("Traffic::Transport: configuration"
                                           " item %s duplicate string: %s:%hu",
                                           item.first.c_str(),
                                           sLabel.c_str(),
                                           nemId);
                }

              if(!nemToLabelMap_.emplace(nemId,sLabel).second)
                {
                  throw ConfigureException("Traffic::Transport: configuration"
                                           " item %s duplicate nem id: %s:%hu",
                                           item.first.c_str(),
                                           sLabel.c_str(),
                                           nemId);
                }

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      INFO_LEVEL,
                                      "TRANSPORTI %03hu"
                                      " Traffic::Transport::%s %s: %s = %hu (%zu bytes)",
                                      id_,
                                      __func__,
                                      item.first.c_str(),
                                      sLabel.c_str(),
                                      nemId,
                                      sLabel.size());
            }

          // is this nem's id in the mapping
          if(const auto iter = nemToLabelMap_.find(id_);
             iter != nemToLabelMap_.end())
            {
              sLocalLabel_ = iter->second;
            }
          else
            {
              throw ConfigureException("Traffic::Transport: configuration"
                                       " item %s missing local nem entry: %hu",
                                       item.first.c_str(),
                                       id_);
            }
        }
      else
        {
          throw ConfigureException("Traffic::Transport: "
                                   "Unexpected configuration item %s",
                                   item.first.c_str());
        }
    }
}


void EMANE::Transports::Traffic::Transport::start()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s",
                          id_,
                          __func__);

  pZMQContext_.reset(zmq_ctx_new());

  pStreamSocket_.reset(zmq_socket(pZMQContext_.get(),ZMQ_STREAM));

  if(!pStreamSocket_)
    {
      throw StartException("Error creating stream socket: %s",
                           zmq_strerror(errno));
    }

  int iIPv6 = 1;

  if(zmq_setsockopt(pStreamSocket_.get(),ZMQ_IPV6,&iIPv6,sizeof(iIPv6)))
    {
      throw StartException("Error setting stream socket opiton ZMQ_IPV6: %s",
                           zmq_strerror(errno));
    }

  if(zmq_bind(pStreamSocket_.get(),
              std::string{"tcp://"}.append(listenEndpoint_.str()).c_str()))
    {
      throw StartException("Error binding stream socket: %s",
                           zmq_strerror(errno));
    }

  size_t fdSize{sizeof(iStreamSocketFd_)};

  if(zmq_getsockopt(pStreamSocket_.get(),ZMQ_FD,&iStreamSocketFd_,&fdSize))
    {
      throw StartException("Error subscribing stream socket fd: %s",
                           zmq_strerror(errno));
    }

  pPlatformService_->fileDescriptorService().
    addFileDescriptor(iStreamSocketFd_,
                      FileDescriptorServiceProvider::DescriptorType::READ,
                      std::bind(&Transport::process,
                                this,
                                std::placeholders::_1));
}

void EMANE::Transports::Traffic::Transport::postStart()
{}

void EMANE::Transports::Traffic::Transport::stop()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s",
                          id_,
                          __func__);

  pPlatformService_->fileDescriptorService().
    removeFileDescriptor(iStreamSocketFd_);
}


void EMANE::Transports::Traffic::Transport::destroy()
  throw()
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s",
                          id_,
                          __func__);
}

void
EMANE::Transports::Traffic::
Transport::processUpstreamPacket(UpstreamPacket & pkt,
                                 const ControlMessages & msgs)
{
  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  if(!msgs.empty())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s does not"
                              " support control messages, ignoring",
                              id_,
                              __func__);
    }

  if(remoteStreamIdentity_.empty())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s"
                              " cannot send Traffic message for src: %hu dst"
                              " %hu not connected, ignoring",
                              id_,
                              __func__,
                              pktInfo.getSource(),
                              pktInfo.getDestination());

      // ignoring
      return;
    }

  auto m = EMANEMessage::Traffic();

  if(!m.ParseFromArray(pkt.get(),pkt.length()))
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s"
                              " unable to parse Traffic src: %hu dst %hu,"
                              " ignoring",
                              id_,
                              __func__,
                              pktInfo.getSource(),
                              pktInfo.getDestination());

      // ignoring
      return;
    }

  m.set_type(EMANEMessage::Traffic::TYPE_UPSTREAM);

  m.set_local(sLocalLabel_);

  m.set_ttl(5);

  std::string sSerialization{};

  if(!m.SerializeToString(&sSerialization))
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s"
                              " unable to serialize Traffic message"
                              " src: %hu dst %hu,"
                              " dropping",
                              id_,
                              __func__,
                              pktInfo.getSource(),
                              pktInfo.getDestination());
    }
  else
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s"
                              " sending Traffic message"
                              " src: %hu dst %hu,"
                              " source: %s"
                              " destination: %s"
                              " local: %s"
                              " priority: %u"
                              " sequence: %zu"
                              " bytes: %zu",
                              id_,
                              __func__,
                              pktInfo.getSource(),
                              pktInfo.getDestination(),
                              m.source().c_str(),
                              m.destination().c_str(),
                              m.local().c_str(),
                              m.priority(),
                              m.sequence_number(),
                              m.payload().size());

      std::uint32_t u32MessageFrameLength = htonl(sSerialization.size());

      // create the message vector starting with length
      std::vector<std::uint8_t> message{&reinterpret_cast<std::uint8_t *>(&u32MessageFrameLength)[0],
                                        &reinterpret_cast<std::uint8_t *>(&u32MessageFrameLength)[4]};

      // insert the reset of the message
      message.insert(message.end(),
                     sSerialization.begin(),
                     sSerialization.end());

      if(zmq_send(pStreamSocket_.get(),
                  remoteStreamIdentity_.data(),
                  remoteStreamIdentity_.size(),
                  ZMQ_SNDMORE) < 0 ||
         zmq_send(pStreamSocket_.get(),
                  message.data(),
                  message.size(),
                  0) < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "TRANSPORTI %03hu Traffic::Transport::%s"
                                  " unable to send Traffic message: %s",
                                  id_,
                                  __func__,
                                  zmq_strerror(errno));
        }
      else
        {
          pPlatformService_->activityService().
            notifyAction("emane.traffic." +
                         sLocalLabel_,
                         m.source() +
                         "-" +
                         m.destination() +
                         "-" +
                         sLocalLabel_ +
                         "-" +
                         std::to_string(m.sequence_number()));
        }

      // must check to see if the stream is readable after a send, from
      // man page for zmq_getsockopt:
      //
      //  The returned file descriptor is also used internally by the
      //  zmq_send and zmq_recv functions. As the descriptor is edge
      //  triggered, applications must update the state of ZMQ_EVENTS
      //  after each invocation of zmq_send or zmq_recv.To be more
      //  explicit: after calling zmq_send the socket may become readable
      //  (and vice versa) without triggering a read event on the file
      //  descriptor.
      int zevents{};
      size_t zeventsSize{sizeof(zevents)};

      zmq_getsockopt(pStreamSocket_.get(),
                     ZMQ_EVENTS,
                     &zevents,
                     &zeventsSize);

      if(zevents & ZMQ_POLLIN)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  DEBUG_LEVEL,
                                  "TRANSPORTI %03hu Traffic::Transport::%s"
                                  " stream ready to read after send",
                                  id_,
                                  __func__);

          process(0);
        }
    }
}


void
EMANE::Transports::Traffic::
Transport::processUpstreamControl(const ControlMessages & msgs)
{
  if(!msgs.empty())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu Traffic::Transport::%s does not"
                              " support control messages, ignoring",
                              id_,
                              __func__);
    }
}

void EMANE::Transports::Traffic::Transport::process(int)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s start",
                          id_,
                          __func__);

  int zevents{};
  size_t zeventsSize{sizeof(zevents)};

  while(1)
    {
      zmq_getsockopt(pStreamSocket_.get(),
                     ZMQ_EVENTS,
                     &zevents,
                     &zeventsSize);

      if(zevents & ZMQ_POLLIN)
        {
          StreamIdentity identity{};

          while(1)
            {
              zmq_msg_t message;

              zmq_msg_init(&message);

              if(zmq_msg_recv(&message,pStreamSocket_.get(),ZMQ_DONTWAIT) == -1)
                {
                  break;
                }

              int iMore{};

              size_t sizeMore{sizeof(iMore)};

              zmq_getsockopt(pStreamSocket_.get(),ZMQ_RCVMORE,&iMore,&sizeMore);

              bool bHasData{};

              if(identity.empty())
                {
                  identity.insert(identity.end(),
                                  reinterpret_cast<std::uint8_t *>(zmq_msg_data(&message)),
                                  reinterpret_cast<std::uint8_t *>(zmq_msg_data(&message)) +
                                  zmq_msg_size(&message));
                }
              else
                {
                  bHasData = zmq_msg_size(&message);
                }

              // when a ZMQ_STREAM TCP session is detected
              // active or dropped you get a zmq message
              // with no data other than the stream
              // identifier
              if(!iMore)
                {
                  if(!bHasData)
                    {
                      if(remoteStreamIdentity_.empty() ||
                         remoteStreamIdentity_ == identity)
                        {
                          if(!remoteStreamIdentity_.empty())
                            {
                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      INFO_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " active connection dropped",
                                                      id_,
                                                      __func__);


                              session_.clear();

                              remoteStreamIdentity_.clear();
                            }
                          else
                            {
                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      INFO_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " active connection established",
                                                      id_,
                                                      __func__);

                              remoteStreamIdentity_ = identity;
                            }
                        }
                      else
                        {
                          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                  ERROR_LEVEL,
                                                  "TRANSPORTI %03hu Traffic::Transport::%s"
                                                  " rejecting new active connection"
                                                  " while currently connected",
                                                  id_,
                                                  __func__);

                          if(zmq_send(pStreamSocket_.get(),
                                      identity.data(),
                                      identity.size(),
                                      ZMQ_SNDMORE) < 0 ||
                             zmq_send(pStreamSocket_.get(),
                                      nullptr,
                                      0,
                                      0) < 0)
                            {
                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      ERROR_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " error closing connection: %s",
                                                      id_,
                                                      __func__,
                                                      zmq_strerror(errno));
                            }
                        }
                    }
                  else
                    {
                      for(auto & entry :
                            session_.process(zmq_msg_data(&message),
                                             zmq_msg_size(&message)))
                        {
                          auto m = EMANEMessage::Traffic();

                          if(!m.ParseFromArray(entry.data(),entry.size()))
                            {
                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      ERROR_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " unable to parse Traffic, ignoring",
                                                      id_,
                                                      __func__);

                              // ignoring
                              return;
                            }

                          if(m.type() == EMANEMessage::Traffic::TYPE_DOWNSTREAM)
                            {
                              bool bDrop{};
                              NEMId source{};
                              NEMId destination{};

                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      DEBUG_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " source: %s"
                                                      " destination: %s"
                                                      " priority: %u"
                                                      " sequence: %zu"
                                                      " bytes: %zu",
                                                      id_,
                                                      __func__,
                                                      m.source().c_str(),
                                                      m.destination().c_str(),
                                                      m.priority(),
                                                      m.sequence_number(),
                                                      m.payload().size());

                              if(const auto iter = labelToNEMMap_.find(m.source());
                                 iter != labelToNEMMap_.end())
                                {
                                  source = iter->second;
                                }
                              else
                                {
                                  bDrop = true;

                                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                          ERROR_LEVEL,
                                                          "TRANSPORTI %03hu Traffic::Transport::%s"
                                                          " unknown NEM id for source %s, dropping",
                                                          id_,
                                                          __func__,
                                                          m.source().c_str());
                                }

                              if(const auto iter = labelToNEMMap_.find(m.destination());
                                 iter != labelToNEMMap_.end())
                                {
                                  destination = iter->second;
                                }
                              else
                                {
                                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                          ERROR_LEVEL,
                                                          "TRANSPORTI %03hu Traffic::Transport::%s"
                                                          " unknown NEM id for destination %s using %hu",
                                                          id_,
                                                          __func__,
                                                          m.destination().c_str(),
                                                          NEM_BROADCAST_MAC_ADDRESS);

                                  destination = NEM_BROADCAST_MAC_ADDRESS;
                                }

                              if(!bDrop)
                                {
                                  // create downstream packet with packet info
                                  DownstreamPacket pkt(PacketInfo(source,
                                                                  destination,
                                                                  m.priority(),
                                                                  Clock::now()),
                                                       entry.data(),
                                                       entry.size());

                                  sendDownstreamPacket(pkt);

                                  pPlatformService_->activityService().
                                    notifyAction("emane.traffic." +
                                                 sLocalLabel_,
                                                 sLocalLabel_ +
                                                 "-" +
                                                 m.destination() +
                                                 "-" +
                                                 sLocalLabel_ +
                                                 "-" +
                                                 std::to_string(m.sequence_number()));

                                }
                            }
                          else
                            {
                              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                      ERROR_LEVEL,
                                                      "TRANSPORTI %03hu Traffic::Transport::%s"
                                                      " wrong message direction, dropping",
                                                      id_,
                                                      __func__);
                            }
                        }
                    }

                  zmq_msg_close(&message);

                  break;
                }
              else
                {
                  zmq_msg_close(&message);
                }
            }
        }
      else
        {
          break;
        }
    }

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL,
                          "TRANSPORTI %03hu Traffic::Transport::%s fin",
                          id_,
                          __func__);

}

DECLARE_TRANSPORT(EMANE::Transports::Traffic::Transport);
