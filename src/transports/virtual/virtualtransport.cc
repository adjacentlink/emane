/*
 * Copyright (c) 2013-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "virtualtransport.h"

#include "emane/downstreampacket.h"
#include "emane/downstreamtransport.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"

#include "emane/utils/netutils.h"

#include "emane/controls/serializedcontrolmessage.h"
#include "emane/controls/flowcontrolcontrolmessage.h"

#include "emane/controls/r2rineighbormetriccontrolmessage.h"
#include "emane/controls/r2rineighbormetriccontrolmessageformatter.h"

#include "emane/controls/r2riqueuemetriccontrolmessage.h"
#include "emane/controls/r2riqueuemetriccontrolmessageformatter.h"

#include "emane/controls/r2riselfmetriccontrolmessage.h"
#include "emane/controls/r2riselfmetriccontrolmessageformatter.h"

#include <sstream>

namespace {
  const std::uint16_t DROP_CODE_WRITE_ERROR = 1;
  const std::uint16_t DROP_CODE_FRAME_ERROR = 2;

  EMANE::StatisticTableLabels STATISTIC_TABLE_LABELS {"Write Error",
                                                      "Frame Error"};
}


EMANE::Transports::Virtual::VirtualTransport::VirtualTransport(NEMId id,
                                                               PlatformServiceProvider * pPlatformService):
  EthernetTransport(id, pPlatformService),
  pTunTap_{},
  pBitPool_{},
  bCanceled_{},
  flowControlClient_{*this},
  bFlowControlEnable_{},
  commonLayerStatistics_{STATISTIC_TABLE_LABELS}
{}

EMANE::Transports::Virtual::VirtualTransport::~VirtualTransport()
{
  if(pTunTap_)
    {
      delete pTunTap_;

      pTunTap_ = nullptr;
    }

  if(pBitPool_)
    {
      delete pBitPool_;

      pBitPool_ = nullptr;
    }
}

void EMANE::Transports::Virtual::VirtualTransport::initialize(Registrar & registrar)
{
  pTunTap_ = new TunTap{pPlatformService_};

  pBitPool_ = new Utils::BitPool{pPlatformService_, id_};

  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<std::string>("device",
                                                  ConfigurationProperties::DEFAULT,
                                                  {"emane0"},
                                                  "Virtual device name.");


  configRegistrar.registerNonNumeric<std::string>("devicepath",
                                                  ConfigurationProperties::DEFAULT,
                                                  {NETWORK_DEVICE_PATH},
                                                  "Path to the tuntap device.");

  configRegistrar.registerNumeric<std::uint64_t>("bitrate",
                                                 ConfigurationProperties::DEFAULT,
                                                 {0},
                                                 "Transport bitrate in bps. This is the total allowable"
                                                 " throughput for the transport combined in both directions"
                                                 " (upstream and downstream). A value of 0 disables the"
                                                 " bitrate feature.");

  configRegistrar.registerNumeric<bool>("broadcastmodeenable",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Broadcast all packets to all NEMs.");


  configRegistrar.registerNumeric<bool>("arpcacheenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Enable ARP request/reply monitoring to map Ethernet address to NEM.");


  configRegistrar.registerNumeric<bool>("arpmodeenable",
                                        ConfigurationProperties::DEFAULT,
                                        {true},
                                        "Enable ARP on the virtual device.");

  configRegistrar.registerNumeric<bool>("flowcontrolenable",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Enables downstream traffic flow control with a corresponding flow"
                                        " control capable NEM layer.");

  configRegistrar.registerNonNumeric<INETAddr>("address",
                                               ConfigurationProperties::NONE,
                                               {},
                                               "IPv4 or IPv6 virutal device address.");

  configRegistrar.registerNonNumeric<INETAddr>("mask",
                                               ConfigurationProperties::NONE,
                                               {},
                                               "IPv4 or IPv6 virutal device addres network mask.");

  auto & statisticRegistrar = registrar.statisticRegistrar();

  commonLayerStatistics_.registerStatistics(statisticRegistrar);

}


void EMANE::Transports::Virtual::VirtualTransport::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "bitrate")
        {
          u64BitRate_ = item.second[0].asUINT64();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %ju",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u64BitRate_);
        }
      else if(item.first == "devicepath")
        {
          sDevicePath_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sDevicePath_.c_str());
        }
      else if(item.first == "device")
        {
          sDeviceName_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sDeviceName_.c_str());
        }
      else if(item.first == "mask")
        {
          mask_ =  item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  mask_.str(false).c_str());
        }
      else if(item.first == "address")
        {
          address_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  address_.str(false).c_str());
        }
      else if(item.first == "arpmodeenable")
        {
          bARPMode_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %d",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bARPMode_);
        }
      else if(item.first == "broadcastmodeenable")
        {
          bBroadcastMode_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %d",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bBroadcastMode_);
        }
      else if(item.first == "arpcacheenable")
        {
          bArpCacheMode_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %d",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bArpCacheMode_);
        }
      else if(item.first == "flowcontrolenable")
        {
          bFlowControlEnable_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s %s: %d",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bFlowControlEnable_);
        }
      else
        {
          throw makeException<ConfigureException>("VirtualTransport: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());

        }
    }
}


void EMANE::Transports::Virtual::VirtualTransport::start()
{
  if(pTunTap_->open(sDevicePath_.c_str(), sDeviceName_.c_str()) < 0)
    {
      std::stringstream ssDescription;
      ssDescription << "could not open tuntap device path "
                    << sDevicePath_
                    << " name "
                    << sDeviceName_
                    << std::ends;
      throw StartException(ssDescription.str());
    }

  if(pTunTap_->set_ethaddr (id_) < 0)
    {
      std::stringstream ssDescription;
      ssDescription << "could not set tuntap eth address "
                    << id_
                    << std::ends;
      throw StartException(ssDescription.str());
    }

  if(pTunTap_->activate(bARPMode_) < 0)
    {
      std::stringstream ssDescription;
      ssDescription << "could not activate tuntap arp mode "
                    << bARPMode_
                    << std::ends;
      throw StartException(ssDescription.str());
    }

  // was an interface address provided
  if(!address_.isAny())
    {
      // set tuntap address/mask
      if(pTunTap_->set_addr(address_, mask_) < 0)
        {
          std::stringstream ssDescription;
          ssDescription << "could not set tuntap address "
                        << address_.str();
          ssDescription << " mask "
                        << mask_.str()
                        << std::ends;
          throw StartException(ssDescription.str());
        }
    }

  bCanceled_ = false;

  pBitPool_->setMaxSize(u64BitRate_);

  pPlatformService_->fileDescriptorService().addFileDescriptor(pTunTap_->get_handle(),
                                                               FileDescriptorServiceProvider::DescriptorType::READ,
                                                               std::bind(&VirtualTransport::readDevice,
                                                                         this,
                                                                         std::placeholders::_1));
}

void EMANE::Transports::Virtual::VirtualTransport::postStart()
{
  if(bFlowControlEnable_)
    {
      flowControlClient_.start();

       LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                              DEBUG_LEVEL,
                              "TRANSPORTI %03hu VirtualTransport::%s sent a flow control"
                              " token request, a handshake response is required to process"
                              " packets",
                              id_,
                              __func__);

    }
}

void EMANE::Transports::Virtual::VirtualTransport::stop()
{
  pPlatformService_->fileDescriptorService().removeFileDescriptor(pTunTap_->get_handle());

  if(bFlowControlEnable_)
    {
      flowControlClient_.stop();
    }

  bCanceled_ = true;

  pTunTap_->deactivate();

  pTunTap_->close();
}


void EMANE::Transports::Virtual::VirtualTransport::destroy()
  throw()
{}


void EMANE::Transports::Virtual::VirtualTransport::processUpstreamPacket(UpstreamPacket & pkt,
                                                                         const ControlMessages & msgs)
{
  const TimePoint beginTime{Clock::now()};

  // frame sanity check
  if(verifyFrame(pkt.get(), pkt.length()) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu VirtualTransport::%s frame error",
                              id_,
                              __func__);
    }

  handleUpstreamControl(msgs);

  commonLayerStatistics_.processInbound(pkt);

  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  const Utils::EtherHeader * pEtherHeader {reinterpret_cast<const Utils::EtherHeader*>(pkt.get())};

  // update arp cache
  updateArpCache(pEtherHeader, pktInfo.getSource());

  iovec iov;

  iov.iov_base = const_cast<char*>(reinterpret_cast<const char*>(pkt.get()));
  iov.iov_len  = pkt.length();

  if(pTunTap_->writev(&iov, 1) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03hu VirtualTransport::%s %s",
                              id_,
                              __func__,
                              strerror(errno));

      commonLayerStatistics_.processOutbound(pkt,
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime),
                                             DROP_CODE_WRITE_ERROR);
    }
  else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "TRANSPORTI %03hu VirtualTransport::%s src %hu, dst %hu, dscp %hhu, length %zu",
                             id_,
                             __func__,
                             pktInfo.getSource(),
                             pktInfo.getDestination(),
                             pktInfo.getPriority(),
                             pkt.length());

      commonLayerStatistics_.processOutbound(pkt,
                                             std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

      // drain the bit pool
      const size_t sizePending = pBitPool_->get(pkt.length() * 8);

      // check for bitpool error
      if(sizePending != 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "TRANSPORTI %03hu VirtualTransport::%s bitpool request error %zd of %zd",
                                  id_,
                                  __func__,
                                  sizePending,
                                  pkt.length() * 8);
        }
    }
}


void EMANE::Transports::Virtual::VirtualTransport::processUpstreamControl(const ControlMessages & msgs)
{
  handleUpstreamControl(msgs);
}



void EMANE::Transports::Virtual::VirtualTransport::handleUpstreamControl(const ControlMessages & msgs)
{
  for(const auto & pMessage : msgs)
    {
      switch(pMessage->getId())
        {
          case Controls::FlowControlControlMessage::IDENTIFIER:
            {
              const auto pFlowControlControlMessage =
                static_cast<const Controls::FlowControlControlMessage *>(pMessage);

              if(bFlowControlEnable_)
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                 DEBUG_LEVEL,
                                                 "TRANSPORTI %03hu VirtualTransport::%s received a flow control"
                                                 " token update %hu tokens",
                                                 id_,
                                                 __func__,
                                                 pFlowControlControlMessage->getTokens());

                  flowControlClient_.processFlowControlMessage(pFlowControlControlMessage);
                }
              else
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          ERROR_LEVEL,
                                          "TRANSPORTI %03hu VirtualTransport::%s received a flow control"
                                          " message but flow control is not enabled",
                                          id_,
                                          __func__);
                }
            }
          break;

        case Controls::R2RINeighborMetricControlMessage::IDENTIFIER:
          {
             const auto pR2RINeighborMetricControlMessage =
                static_cast<const Controls::R2RINeighborMetricControlMessage *>(pMessage);


            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::R2RINeighborMetricControlMessageFormatter(pR2RINeighborMetricControlMessage),
                                            "TRANSPORTI %03hu VirtualTransport::%s R2RINeighborMetricControlMessage",
                                            id_,
                                            __func__);
          }
          break;

        case Controls::R2RIQueueMetricControlMessage::IDENTIFIER:
          {
            const auto pR2RIQueueMetricControlMessage =
              static_cast<const Controls::R2RIQueueMetricControlMessage *>(pMessage);

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::R2RIQueueMetricControlMessageFormatter(pR2RIQueueMetricControlMessage),
                                            "TRANSPORTI %03hu VirtualTransport::%s R2RIQueueMetricControlMessage",
                                            id_,
                                            __func__);
          }
          break;

        case Controls::R2RISelfMetricControlMessage::IDENTIFIER:
          {
            const auto pR2RISelfMetricControlMessage =
              static_cast<const Controls::R2RISelfMetricControlMessage *>(pMessage);

            LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            Controls::R2RISelfMetricControlMessageFormatter(pR2RISelfMetricControlMessage),
                                            "TRANSPORTI %03hu VirtualTransport::%s R2RISelfMetricControlMessage",
                                            id_,
                                            __func__);
          }
          break;



          case Controls::SerializedControlMessage::IDENTIFIER:
            {
              const auto pSerializedControlMessage =
                static_cast<const Controls::SerializedControlMessage *>(pMessage);

              switch(pSerializedControlMessage->getSerializedId())
                {
                  case Controls::FlowControlControlMessage::IDENTIFIER:
                    {
                      std::unique_ptr<Controls::FlowControlControlMessage>
                        pFlowControlControlMessage{
                        Controls::FlowControlControlMessage::create(
                                                                pSerializedControlMessage->getSerialization())};

                      if(bFlowControlEnable_)
                        {
                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                 ERROR_LEVEL,
                                                 "TRANSPORTI %03hu VirtualTransport::%s received a flow control"
                                                 " token update %hu tokens",
                                                 id_,
                                                 __func__,
                                                 pFlowControlControlMessage->getTokens());

                            flowControlClient_.processFlowControlMessage(pFlowControlControlMessage.get());

                        }
                      else
                        {
                          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                                  ERROR_LEVEL,
                                                  "TRANSPORTI %03hu VirtualTransport::%s received a flow control"
                                                  " message but flow control is not enabled",
                                                  id_,
                                                  __func__);
                        }
                    }
                  break;

                  case Controls::R2RINeighborMetricControlMessage::IDENTIFIER:
                    {
                      std::unique_ptr<Controls::R2RINeighborMetricControlMessage>
                        pR2RINeighborMetricControlMessage{
                        Controls::R2RINeighborMetricControlMessage::create(
                                                                pSerializedControlMessage->getSerialization())};

                      LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                                      DEBUG_LEVEL,
                                                      Controls::R2RINeighborMetricControlMessageFormatter(
                                                       pR2RINeighborMetricControlMessage.get()),
                                                      "TRANSPORTI %03hu VirtualTransport::%s",
                                                      id_,
                                                      __func__);
                    }
                  break;

                  case Controls::R2RIQueueMetricControlMessage::IDENTIFIER:
                    {
                      std::unique_ptr<Controls::R2RIQueueMetricControlMessage>
                        pR2RIQueueMetricControlMessage{
                        Controls::R2RIQueueMetricControlMessage::create(
                                                                pSerializedControlMessage->getSerialization())};

                      LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                                      DEBUG_LEVEL,
                                                      Controls::R2RIQueueMetricControlMessageFormatter(
                                                       pR2RIQueueMetricControlMessage.get()),
                                                      "TRANSPORTI %03hu VirtualTransport::%s",
                                                      id_,
                                                      __func__);
                    }
                  break;

                  case Controls::R2RISelfMetricControlMessage::IDENTIFIER:
                    {
                      std::unique_ptr<Controls::R2RISelfMetricControlMessage>
                        pR2RISelfMetricControlMessage{
                        Controls::R2RISelfMetricControlMessage::create(
                                                                pSerializedControlMessage->getSerialization())};

                      LOGGER_VERBOSE_LOGGING_FN_VARGS(pPlatformService_->logService(),
                                                      DEBUG_LEVEL,
                                                      Controls::R2RISelfMetricControlMessageFormatter(
                                                       pR2RISelfMetricControlMessage.get()),
                                                      "TRANSPORTI %03hu VirtualTransport::%s",
                                                      id_,
                                                      __func__);
                    }
                  break;

                  default:
                     LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            "TRANSPORTI %03hu VirtualTransport::%s unknown serialized msg id %hu, ignore",
                                            id_,
                                            __func__,
                                            pSerializedControlMessage->getSerializedId());

                }
            }
          break;

          default:
               LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "TRANSPORTI %03hu VirtualTransport::%s unknown msg id %hu, ignore",
                                      id_,
                                      __func__,
                                      pMessage->getId());
      }
   }
}


void EMANE::Transports::Virtual::VirtualTransport::readDevice(int)
{
  std::uint8_t buf[Utils::IP_MAX_PACKET];

  while(!bCanceled_)
    {
      ssize_t len{};

      memset(buf, 0x0, sizeof(buf));

      iovec iov;

      iov.iov_base = reinterpret_cast<char*>(buf);
      iov.iov_len  = sizeof(buf);

      // read from tuntap
      if((len = pTunTap_->readv(&iov, 1)) < 0)
        {
          if(errno != EAGAIN)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      ERROR_LEVEL,
                                      "TRANSPORTI %03hu VirtualTransport::%s %s",
                                      id_,
                                      __func__,
                                      strerror(errno));
            }

          return;
        }
      else
        {
          const TimePoint beginTime{Clock::now()};

          // frame sanity check
          if(verifyFrame(buf, len) < 0)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      ERROR_LEVEL,
                                      "TRANSPORTI %03hu VirtualTransport::%s frame error",
                                      id_,
                                      __func__);
            }
          else
            {
              // NEM destination
              NEMId nemDestination;

              // pkt tos/qos converted to dscp
              std::uint8_t dscp{};

              // get dst and dscp values
              if(parseFrame((const Utils::EtherHeader *)buf, nemDestination, dscp) < 0)
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          ERROR_LEVEL,
                                          "TRANSPORTI %03hu VirtualTransport::%s frame parse error",
                                          id_,
                                          __func__);
                }
              else
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         "TRANSPORTI %03hu VirtualTransport::%s src %hu, dst %hu, dscp %hhu, length %zd",
                                         id_,
                                         __func__,
                                         id_,
                                         nemDestination,
                                         dscp,
                                         len);

                  // create downstream packet with packet info
                  DownstreamPacket pkt(PacketInfo (id_, nemDestination, dscp,Clock::now()), buf, len);

                  commonLayerStatistics_.processInbound(pkt);

                  // check flow control
                  if(bFlowControlEnable_)
                    {
                      auto status = flowControlClient_.removeToken();

                      // block and wait for an available flow control token
                      if(!status.second)
                        {
                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                 ERROR_LEVEL,
                                                 "TRANSPORTI %03hu VirtualTransport::%s failed to remove token (tokens:%hu)",
                                                 id_,
                                                 __func__,
                                                 status.first);
                          // done
                          return;
                        }
                      else
                        {
                          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                                 DEBUG_LEVEL,
                                                 "TRANSPORTI %03hu VirtualTransport::%s removed token (tokens:%hu)",
                                                 id_,
                                                 __func__,
                                                 status.first);
                        }
                    }

                  commonLayerStatistics_.processOutbound(pkt,
                                                         std::chrono::duration_cast<Microseconds>(Clock::now() - beginTime));

                  // send to downstream transport
                  sendDownstreamPacket(pkt);

                  // drain the bit pool
                  std::uint64_t sizePending {pBitPool_->get(len * 8)};

                  // check for bitpool error
                  if(sizePending > 0)
                    {
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              ERROR_LEVEL,
                                              "TRANSPORTI %03hu VirtualTransport::%s bitpool "
                                              "request error %jd of %zd",
                                              id_,
                                              __func__,
                                              sizePending,
                                              len * 8);
                    }
                }
            }
        }
    }
}



DECLARE_TRANSPORT(EMANE::Transports::Virtual::VirtualTransport);
