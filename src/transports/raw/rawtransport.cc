/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <ace/OS_NS_Thread.h>

#ifdef WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#endif

#include "rawtransport.h"

#include "emane/downstreampacket.h"
#include "emane/downstreamtransport.h"
#include "emane/configureexception.h"
#include "emane/startexception.h"

#include "emane/utils/spawnmemberfunc.h"
#include "emane/controls/serializedcontrolmessage.h"

#include <sstream>

namespace
{
  // common
  const int PCAP_SNAPLEN = 0xFFFF;
  const int PCAP_PROMISC = 1;

#ifdef WIN32

  const std::string DEVICE_PREFIX = "\\Device\\NPF_";

  // win32 will buffer packets if PCAP_TIMEOUT is 0
  const int PCAP_TIMEOUT = 1;

#elif defined(__APPLE__)

  const int AddressType = AF_LINK;

  const std::string DEVICE_PREFIX = "";

  // mac will buffer packets if PCAP_TIMEOUT is 0
  const int PCAP_TIMEOUT = 1;

  struct sockaddr_ll_t {
    std::uint8_t  sll_len;
    std::uint8_t  sll_family;
    std::uint16_t sll_ifindex;
    std::uint8_t  sll_type;
    std::uint8_t  sll_nlen;
    std::uint8_t  sll_alen;
    std::uint8_t  sll_slen;
    std::uint8_t  sll_addr[12];
  }__attribute__((packed));

#elif defined(__linux__)

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

#endif 
}


EMANE::Transports::Raw::RawTransport::RawTransport(NEMId id,
                                                   PlatformServiceProvider * pPlatformService):
  EthernetTransport(id, pPlatformService),
  threadRead_{},
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

#ifdef WIN32

  // we would prefer to use libpcap to obtain the interface info
  // but the mac address does not seem to be supported in win32
  // and we need the mac address to check for our own transmissions
  // since winpcap does not support pcap_setdirection
  
  // query the adapter list here and search based on the ip address in win32
  // instead of the complex adapter name like {11661990-8F30-46AD-A4F57D7AC1D2}

  // adpater buff
  IP_ADAPTER_INFO ai[256];

  // buff len
  ULONG buflen = sizeof(ai);

  // adapter list
  IP_ADAPTER_INFO* aip = &ai[0];

  // get all adpater info
  if(GetAdaptersInfo(aip, &buflen) == NO_ERROR) 
    {
      // each adpater
      while (aip && !bMacResolved) 
        {
          // ip address
          IP_ADDR_STRING *ip = &aip->IpAddressList;

          // each ip address
          while(ip && !bMacResolved) 
            {
              // address match
              if(sTargetDevice_ == ip->IpAddress.String) 
                {
                  // add adapter name to device prefix
                  sDeviceName += aip->AdapterName;

                  // save mac addr
                  memcpy(macAddr_.bytes.buff, aip->Address, Utils::ETH_ALEN);

                  LOGGER_STANDARD_LOGGING(pPlatformService_, 
                                          DEBUG_LEVEL, 
                                          "TRANSPORTI %03d RawTransport %s adapter %s, hw addr %s",
                                          id_, 
                                          __func__,
                                          aip->AdapterName,
                                          ethaddr_to_string(&macAddr_).c_str());

                  // set resolved flag
                  bMacResolved = true;
                }
              // next address
              ip = ip->Next;
            }
          // next adapter 
          aip = aip->Next;
        }
    }
  else 
    {
      std::stringstream ssDescription;
      ssDescription<<"could not get adapters list "<< sTargetDevice_ << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

#elif defined(__linux__) || defined(__APPLE__)

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

#ifdef __APPLE__
                  // save mac addr
                  memcpy(macAddr_.bytes.buff, &s->sll_addr[s->sll_nlen], Utils::ETH_ALEN);
#elif defined(__linux__)
                  memcpy(macAddr_.bytes.buff, &s->sll_addr[0], Utils::ETH_ALEN);
#endif

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
#else
#error "unknown platfrom"
#endif


  // free interface list
  pcap_freealldevs(iflist);

  // mac address not resolved
  if(bMacResolved == false)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not resolve our mac address "<< sTargetDevice_ << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // open pcap handle
  if((pPcapHandle_ = pcap_open_live(sDeviceName.c_str(), PCAP_SNAPLEN, PCAP_PROMISC, PCAP_TIMEOUT, errbuf)) == NULL)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not open device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

  // set datalink type, this covers 10/100/1000
  if(pcap_set_datalink(pPcapHandle_, DLT_EN10MB) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set datalink type on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

#ifndef WIN32

  // currently unsupported by winpcap
  if(pcap_setdirection(pPcapHandle_, PCAP_D_IN) < 0)
    {
      std::stringstream ssDescription;
      ssDescription<<"could not set direction on device "<< sDeviceName << " " << errbuf<<std::ends;
      throw StartException(ssDescription.str());
    }

#endif

  pBitPool_->setMaxSize(u64BitRate_);

  // start pcap read thread
  Utils::spawn(*this, &EMANE::Transports::Raw::RawTransport::readDevice, &threadRead_);
}



void EMANE::Transports::Raw::RawTransport::stop()
{
  // cancel read thread
  if(threadRead_ != 0)
    {
      ACE_OS::thr_cancel(threadRead_);

      ACE_OS::thr_join(threadRead_,0,0);

      threadRead_ = 0;
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

#ifdef WIN32

  // lock mutex for writting
  mutex_.acquire_write();

  // add src mac addr to history set since we can not determine packet direction in win32
  upstreamHostSrcMacAddrHistorySet_.insert(*Utils::get_srcaddr(pEtherHeader));

  // unlock mutex
  mutex_.release();

#endif

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



ACE_THR_FUNC_RETURN EMANE::Transports::Raw::RawTransport::readDevice()
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
                  // check for our own outbound transmission
                  if(isOutbound(pEtherHeader))
                    {
                      // skip
                      continue;
                    }

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

  return (ACE_THR_FUNC_RETURN) 0;
}


int EMANE::Transports::Raw::RawTransport::isOutbound(const Utils::EtherHeader *pEtherHeader)
{
#ifdef WIN32

  // check mac src address since we can not determine packet direction in win32

  // lock mutex for reading
  mutex_.acquire_read();

  // packet is from self
  if(memcmp(Utils::get_srcaddr(pEtherHeader), &macAddr_, Utils::ETH_ALEN) == 0)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_, 
                             DEBUG_LEVEL, 
                             "TRANSPORTI %03d RawTransport %s src %s is us",
                             id_, 
                             __func__,
                             ethaddr_to_string(Utils::get_srcaddr(pEtherHeader)).c_str());

      // yes src is us
      return 1;
    }

  // check the src mac addr with the upstream src mac addr set 
  EthAddrSetConstIter iter = upstreamHostSrcMacAddrHistorySet_.find(pEtherHeader->src);

  // this source was seen on the upstream path
  if(iter != upstreamHostSrcMacAddrHistorySet_.end())
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_, 
                             DEBUG_LEVEL, 
                             "TRANSPORTI %03d RawTransport %s src %s is downstream",
                             id_, 
                             __func__,
                             ethaddr_to_string(&pEtherHeader->src).c_str());
 
      // yes this is from a remote NEM and the frame is heading up and out to ip stack
      return 1;
    }

  // unlock mutex
  mutex_.release();

#else
  // only needed in windoz nothing to do here
  ACE_UNUSED_ARG(pEtherHeader);

#endif

  // no, its downstream (from ip)
  return 0;
}


DECLARE_TRANSPORT(EMANE::Transports::Raw::RawTransport);
