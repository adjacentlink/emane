/*
 * Copyright (c) 2013-2014,2016,2023 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "rawtransport.h"

#include "emane/downstreampacket.h"
#include "emane/downstreamtransport.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"

#include "emane/utils/threadutils.h"
#include "emane/utils/parameterconvert.h"

#include "emane/controls/serializedcontrolmessage.h"

#include <sstream>

namespace
{
  const int PCAP_SNAPLEN = 0xFFFF;
  const int PCAP_PROMISC = 1;
  const int PCAP_IMMEDIATE = 1;

  const int AddressType = AF_PACKET;

  const std::string DEVICE_PREFIX = "";

  const int PCAP_TIMEOUT = 0;

  struct sockaddr_ll_t {
    std::uint16_t sll_family;
    std::uint16_t sll_protocol;
    std::uint32_t sll_ifindex;
    std::uint16_t sll_type;
    std::uint8_t  sll_pkttype;
    std::uint8_t  sll_halen;
    std::uint8_t  sll_addr[8];
  }__attribute__((packed));
}


EMANE::Transports::Raw::RawTransport::RawTransport(NEMId id,
                                                   PlatformServiceProvider * pPlatformService):
  EthernetTransport(id, pPlatformService),
  thread_{},
  pPcapHandle_{},
  pBitPool_{},
  u64BitRate_{}
{
  memset(&macAddr_, 0x0, sizeof(macAddr_));
}

EMANE::Transports::Raw::RawTransport::~RawTransport()
{
  // close pcap handle
  // Moves pcap_close call to the destructor
  // instead of the stop call because having it
  // there caused emanetransportd to hang on a
  // ctrl-c
  if (pPcapHandle_ != NULL)
    {
      pcap_close(pPcapHandle_);

      pPcapHandle_ = NULL;
    }

  if(pBitPool_ != NULL)
    {
      delete pBitPool_;

      pBitPool_ = NULL;
    }
}


void EMANE::Transports::Raw::RawTransport::initialize(Registrar & registrar)
{
  // create bit pool
  pBitPool_ = new Utils::BitPool(pPlatformService_, id_);

  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<std::string>("device",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Device to use as the raw packet entry point.");

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

  configRegistrar.registerNonNumeric<std::string>("ethernet.type.unknown.priority",
                                                  ConfigurationProperties::NONE,
                                                  {},
                                                  "Defines the emulator priority value (DSCP"
                                                  " used for IP) to use when the specified"
                                                  " unknown Ethernet type is encountered"
                                                  " during downstream processing. Uses the"
                                                  " following format: <ethernet type>:<priority>.",
                                                  0,
                                                  std::numeric_limits<std::uint16_t>::max(),
                                                  "^(0[xX]){0,1}\\d+:\\d+$");

  configRegistrar.registerNumeric<std::uint8_t>("ethernet.type.arp.priority",
                                                ConfigurationProperties::DEFAULT,
                                                {0},
                                                "Defines the emulator priority value (DSCP"
                                                " used for IP) to use when an ARP Ethernet"
                                                " frame is encountered during downstream"
                                                " processing.");
}



void EMANE::Transports::Raw::RawTransport::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "bitrate")
        {
          // value is in bits
          u64BitRate_ =  item.second[0].asUINT64();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu Transports::Raw::RawTransport %s %s: %ju",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u64BitRate_);
        }
      else if(item.first == "device")
        {
          sTargetDevice_ = item.second[0].asString();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu RawTransport %s %s: %s",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  sTargetDevice_.c_str());
        }
      else if(item.first == "broadcastmodeenable")
        {
          bBroadcastMode_ = item.second[0].asBool();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03d RawTransport %s %s: %d",
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
                                  "TRANSPORTI %03d RawTransport %s %s: %d",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  bArpCacheMode_);
        }
      else if(item.first == "ethernet.type.arp.priority")
        {
          u8EtherTypeARPPriority_ = item.second[0].asUINT8();

          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  INFO_LEVEL,
                                  "TRANSPORTI %03hu RawTransport::%s %s: %hhu",
                                  id_,
                                  __func__,
                                  item.first.c_str(),
                                  u8EtherTypeARPPriority_);
        }
      else if(item.first == "ethernet.type.unknown.priority")
        {
          for(const auto & value : item.second)
            {
              std::string sEntry{value.asString()};

              auto pos = sEntry.find_first_of(':');

              std::uint16_t u16EtherType =
                Utils::ParameterConvert(sEntry.substr(0,pos)).toUINT16();

              std::int8_t u8Priority =
                Utils::ParameterConvert(sEntry.substr(pos+1)).toUINT8();

              unknownEtherTypePriorityMap_[u16EtherType] = u8Priority;

              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      INFO_LEVEL,
                                      "TRANSPORTI %03hu RawTransport::%s %s: %#hx:%hhu",
                                      id_,
                                      __func__,
                                      item.first.c_str(),
                                      u16EtherType,
                                      u8Priority);

            }
        }
      else
        {
          throw makeException<ConfigureException>("RawTransport: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());

        }
    }
}



void EMANE::Transports::Raw::RawTransport::start()
{
  bool bMacResolved{false};

  // pcap error buff
  char errbuf[PCAP_ERRBUF_SIZE]={};

  // pcap interface list
  struct pcap_if *iflist;

  // device name
  std::string sDeviceName = DEVICE_PREFIX;

  // get all available interfaces
  if (pcap_findalldevs (&iflist, errbuf) < 0 || iflist == NULL)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not get interface list "<< errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }


  // each interface
  struct pcap_if *ifp = iflist;

  // walk through all available interfaces
  while (ifp)
    {
      // name match
      if(sTargetDevice_ == ifp->name)
        {
          // add adapter name to device prefix
          sDeviceName += ifp->name;

          // address list
          pcap_addr_t *ap = ifp->addresses;

          // for each address
          while (ap && !bMacResolved)
            {
              // mac addr
              if(ap->addr->sa_family == AddressType)
                {
                  struct sockaddr_ll_t *s = (struct sockaddr_ll_t *) ap->addr;

                  memcpy(macAddr_.bytes.buff, &s->sll_addr[0], Utils::ETH_ALEN);

                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          DEBUG_LEVEL,
                                          "TRANSPORTI %03d RawTransport %s adapter %s, hw addr %s",
                                          id_,
                                          __func__,
                                          ifp->name,
                                          ethaddr_to_string(&macAddr_).c_str());

                  // set resolved flag
                  bMacResolved = true;
                }
              // next addr
              ap = ap->next;
            }
        }
      // next interface
      ifp = ifp->next;
    }

  // free interface list
  pcap_freealldevs(iflist);

  // mac address not resolved
  if(bMacResolved == false)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not resolve our mac address "<< sTargetDevice_ << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // create pcap handle
  if((pPcapHandle_ = pcap_create(sDeviceName.c_str(), errbuf)) == NULL)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not create pcap handle for device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // pre-activate immediate mode, packets are always delivered as soon as they arrive, with no buffering.
  if(pcap_set_immediate_mode(pPcapHandle_, PCAP_IMMEDIATE) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set pcap immediate mode type on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // pre-activate set snap len
  if(pcap_set_snaplen(pPcapHandle_, PCAP_SNAPLEN) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set pcap snaplen on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // pre-activate set promisc
  if(pcap_set_promisc(pPcapHandle_, PCAP_PROMISC) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set pcap promisc device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // activate
  if(pcap_activate(pPcapHandle_) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not activate pcap handle on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // post-activate set datalink type, this covers 10/100/1000
  if(pcap_set_datalink(pPcapHandle_, DLT_EN10MB) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set pcap datalink type on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // post-activate currently unsupported by winpcap
  if(pcap_setdirection(pPcapHandle_, PCAP_D_IN) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set pcap direction on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  pBitPool_->setMaxSize(u64BitRate_);

  // start pcap read thread
  thread_ = std::thread(&RawTransport::readDevice,this);
}



void EMANE::Transports::Raw::RawTransport::stop()
{
  // cancel read thread
  if(thread_.joinable())
    {
      ThreadUtils::cancel(thread_);

      thread_.join();
    }
}


void EMANE::Transports::Raw::RawTransport::destroy()
  throw()
{}



void EMANE::Transports::Raw::RawTransport::processUpstreamPacket(UpstreamPacket & pkt,
                                                                 const ControlMessages &msgs)
{
  // we are not in the running state - the infrastructure should protect
  // against this but currently it does not for transports.
  if(!pPcapHandle_)
    {
      return;
    }

  // frame sanity check
  if(verifyFrame(pkt.get(), pkt.length()) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03d RawTransport %s ethernet frame error",
                              id_,
                              __func__);
    }

  handleUpstreamControl(msgs);

  // get packet info
  const PacketInfo & pktInfo{pkt.getPacketInfo()};

  // ether header
  const Utils::EtherHeader * pEtherHeader = (const Utils::EtherHeader*) pkt.get();

  // update arp cache
  updateArpCache(pEtherHeader, pktInfo.getSource());

  // send packet
  if(pcap_sendpacket(pPcapHandle_, (const std::uint8_t*) pkt.get(), pkt.length()) < 0)
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ERROR_LEVEL,
                              "TRANSPORTI %03d RawTransport %s pcap_sendpacket error %s",
                              id_,
                              __func__,
                              pcap_geterr(pPcapHandle_));
    }
  else
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             DEBUG_LEVEL,
                             "TRANSPORTI %03d RawTransport %s src %hu, dst %hu, dscp %hhu, length %zu",
                             id_,
                             __func__,
                             pktInfo.getSource(),
                             pktInfo.getDestination(),
                             pktInfo.getPriority(),
                             pkt.length());

      // drain the bit pool converting bytes to bits
      const size_t sizePending = pBitPool_->get(pkt.length() * 8);

      // check for bitpool error
      if(sizePending != 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "TRANSPORTI %03d RawTransport %s bitpool request error %zd of %zd",
                                  id_,
                                  __func__,
                                  sizePending, pkt.length() * 8);
        }
    }
}


void EMANE::Transports::Raw::RawTransport::processUpstreamControl(const ControlMessages & msgs)
{
  handleUpstreamControl(msgs);
}

void EMANE::Transports::Raw::RawTransport::handleUpstreamControl(const ControlMessages & msgs)
{
  for(const auto & pMessage : msgs)
    {
      switch(pMessage->getId())
        {
          case Controls::SerializedControlMessage::IDENTIFIER:
            {
              const auto pSerializedControlMessage =
                static_cast<const Controls::SerializedControlMessage *>(pMessage);

              switch(pSerializedControlMessage->getSerializedId())
                {
                  default:
                     LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                            DEBUG_LEVEL,
                                            "TRANSPORTI %03hu RawTransport::%s unknown serialized msg id %hu, ignore",
                                            id_,
                                            __func__,
                                            pSerializedControlMessage->getSerializedId());

                }
            }
          break;

          default:
               LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                      DEBUG_LEVEL,
                                      "TRANSPORTI %03hu RawTransport::%s unknown msg id %hu, ignore",
                                      id_,
                                      __func__,
                                      pMessage->getId());
      }
   }
}



void EMANE::Transports::Raw::RawTransport::readDevice()
{
  const std::uint8_t* buf = NULL;

  struct pcap_pkthdr *pcap_hdr = NULL;

  int iPcapResult{};

  while(1)
    {
      // get frame, blocks here
      iPcapResult = pcap_next_ex(pPcapHandle_, &pcap_hdr, &buf);

      // error
      if (iPcapResult < 0)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  ERROR_LEVEL,
                                  "TRANSPORTI %03d RawTransport %s pcap_next_ex error %s",
                                  id_,
                                  __func__,
                                  pcap_geterr(pPcapHandle_));

          // done
          break;
        }
      // time out
      else if (iPcapResult == 0)
        {
          continue;
        }
      // success
      else if(iPcapResult == 1)
        {
          // frame sanity check
          if(verifyFrame(buf, pcap_hdr->caplen) < 0)
            {
              LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                      ERROR_LEVEL,
                                      "TRANSPORTI %03d RawTransport %s frame error",
                                      id_,
                                      __func__);
            }
          else
            {
              const Utils::EtherHeader *pEtherHeader = (const Utils::EtherHeader *) buf;

              NEMId nemDestination;

              std::uint8_t dscp{};

              // get dst and dscp values from frame
              if(parseFrame(pEtherHeader, nemDestination, dscp) < 0)
                {
                  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                          ERROR_LEVEL,
                                          "TRANSPORTI %03d RawTransport %s frame parse error",
                                          id_,
                                          __func__);
                }
              else
                {
                  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                         DEBUG_LEVEL,
                                         "TRANSPORTI %03d RawTransport %s src %hu, dst %hu, dscp %hhu, length %u",
                                         id_,
                                         __func__,
                                         id_,
                                         nemDestination,
                                         dscp,
                                         pcap_hdr->caplen);

                  // create downstream packet with packet info
                  DownstreamPacket pkt(PacketInfo (id_, nemDestination, dscp,Clock::now()), buf, pcap_hdr->caplen);

                  sendDownstreamPacket(pkt);

                  // drain the bit pool converting bytes to bits
                  const size_t sizePending = pBitPool_->get(pcap_hdr->caplen * 8);

                  // check for bitpool error
                  if(sizePending != 0)
                    {
                      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                              ERROR_LEVEL,
                                              "TRANSPORTI %03d RawTransport %s bitpool request error %zd of %u",
                                              id_,
                                              __func__,
                                              sizePending,
                                              pcap_hdr->caplen * 8);
                    }
                }
            }
        }
    }
}


DECLARE_TRANSPORT(EMANE::Transports::Raw::RawTransport);
