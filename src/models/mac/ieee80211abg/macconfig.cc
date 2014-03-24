/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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


#include "macconfig.h"
#include "utils.h"

#include "emane/utils/parameterconvert.h"

#include <ace/Guard_T.h>

#include <vector>

namespace
{
  // valid data rate index [1, 12], the leading 0 is only used to align the index
  // unicast
  const std::vector<std::uint32_t> UnicastDataRateIndexTable
         { 0, 1000, 2000, 5500, 11000, 6000, 9000, 12000, 18000, 24000, 36000, 48000, 54000 };

  // multicast/broadcast
  const std::vector<std::uint32_t> BroadcastDataRateIndexTable
         { 0, 1000, 2000, 5500, 11000, 6000, 9000, 12000, 18000, 24000, 36000, 48000, 54000 };

  const char * pzLayerName {"MACConfig"};
}

/**
*
* @brief ieee80211abg mac configuration initializer.
*
*/
EMANE::Models::IEEE80211ABG::ConfigItems::ConfigItems()
{ }


/**
*
* @brief constructor
*
*/
EMANE::Models::IEEE80211ABG::MACConfig::MACConfig(LogServiceProvider & logServiceProvider, NEMId id) :
 logServiceProvider_(logServiceProvider),
 id_{id}
{ }
 

/**
*
* @brief destructor
*
*/
EMANE::Models::IEEE80211ABG::MACConfig::~MACConfig()
{ }


void EMANE::Models::IEEE80211ABG::MACConfig::registerConfiguration(ConfigurationRegistrar & configRegistrar)
{

  configRegistrar.registerNumeric<bool>("enablepromiscuousmode",
                                        ConfigurationProperties::DEFAULT |
                                        ConfigurationProperties::MODIFIABLE,
                                        {false},
                                        "Defines whether promiscuous mode is enabled or not. If promiscuous"
                                        " mode is enabled, all received packets (intended for the given"
                                        " node or not) that pass the probability of reception check are"
                                        " sent upstream to the transport.");

  configRegistrar.registerNumeric<bool>("wmmenable",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines if wireless multimedia mode (WMM) is enabled.");

  configRegistrar.registerNumeric<std::uint8_t>("mode",
                                                ConfigurationProperties::DEFAULT,
                                                {0},
                                                "Defines the 802.11abg mode of operation.  0|2 = 802.11b"
                                                " (DSS), 1 = 802.11a/g (OFDM), and 3 = 802.11b/g"
                                                " (mixed mode).",
                                                MODULATION_TYPE_INDEX_MIN,
                                                MODULATION_TYPE_INDEX_MAX);

  configRegistrar.registerNumeric<std::uint8_t>("unicastrate",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {4},
                                                "Defines the data rate to be used when transmitting unicast"
                                                " packets. The index (1 through 12) to rate (Mbps) mapping is as"
                                                " follows: [1 2 5.5 11 6 9 12 18 24 36 48 54]. DSS rates [1 2"
                                                " 5.5 11] Mbps are valid when mode is set to 802.11b or 802.11b/g."
                                                " OFDM rates [6 9 12 18 24 36 48 54] Mbps are valid when mode is"
                                                " set to 802.11a/g or 802.11b/g.",
                                                 1, 
                                                12);

  configRegistrar.registerNumeric<std::uint8_t>("multicastrate",
                                                ConfigurationProperties::DEFAULT |
                                                ConfigurationProperties::MODIFIABLE,
                                                {1},
                                                "Defines the data rate to be used when transmitting"
                                                " broadcast/multicast packets. The index (1 through 12) to rate"
                                                " (Mbps) mapping is as follows: [1 2 5.5 11 6 9 12 18 24 36 48"
                                                " 54].  DSS rates [1 2 5.5 11] Mbps are valid when mode is set to"
                                                " 802.11b or 802.11b/g.  OFDM rates [6 9 12 18 24 36 48 54] Mbps"
                                                " are valid when mode is set to 802.11a/g or 802.11b/g.",
                                                 1, 
                                                 12);

  configRegistrar.registerNumeric<std::uint16_t>("rtsthreshold",
                                                 ConfigurationProperties::DEFAULT,
                                                 {255},
                                                 "Defines a threshold in bytes for when RTS/CTS is used as part of"
                                                 " the carrier sensing channel access protocol when transmitting"
                                                 " unicast packets.",
                                                 0, 
                                                 0xffff);

  configRegistrar.registerNumeric<bool>("flowcontrolenable",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines whether flow control is enabled. Flow control only"
                                        " works with the virtual transport and the setting must"
                                        " match the setting within the virtual transport"
                                        " configuration.");

  configRegistrar.registerNumeric<std::uint16_t>("flowcontroltokens",
                                                 ConfigurationProperties::DEFAULT,
                                                 {10},
                                                 "Defines the maximum number of flow control tokens"
                                                 " (packet transmission units) that can be processed"
                                                 " from the virtual transport without being refreshed."
                                                 " The number of available tokens at any given time is"
                                                 " coordinated with the virtual transport and when the"
                                                 " token count reaches zero, no further packets are"
                                                 " transmitted causing application socket queues to backup.",
                                                 1);

  configRegistrar.registerNumeric<std::uint32_t>("distance",
                                                 ConfigurationProperties::DEFAULT,
                                                 {1000},
                                                 "Defines the max propagation distance in meters used"
                                                 " to compute slot size.");

  configRegistrar.registerNonNumeric<std::string>("pcrcurveuri",
                                                  ConfigurationProperties::REQUIRED,
                                                  {},
                                                  "Defines the absolute URI of the Packet Completion Rate (PCR)"
                                                  " curve file. The PCR curve file contains probability of"
                                                  " reception curves as a function of Signal to Interference"
                                                  " plus Noise Ratio (SINR) for each data rate.");

  configRegistrar.registerNumeric<std::uint8_t>("queuesize0",
                                                  ConfigurationProperties::DEFAULT,
                                                  {QUEUE_SIZE_DEFAULT},
                                                  "Defines the queue size for category 0.");

  configRegistrar.registerNumeric<std::uint8_t>("queuesize1",
                                                  ConfigurationProperties::DEFAULT,
                                                  {QUEUE_SIZE_DEFAULT},
                                                  "Defines the queue size for category 1.");

  configRegistrar.registerNumeric<std::uint8_t>("queuesize2",
                                                  ConfigurationProperties::DEFAULT,
                                                  {QUEUE_SIZE_DEFAULT},
                                                  "Defines the queue size for category 2.");

  configRegistrar.registerNumeric<std::uint8_t>("queuesize3",
                                                  ConfigurationProperties::DEFAULT,
                                                  {QUEUE_SIZE_DEFAULT},
                                                  "Defines the queue size for category 3.");

  configRegistrar.registerNumeric<std::uint16_t>("msdu0",
                                                  ConfigurationProperties::DEFAULT,
                                                  {MAX_PACKET_SIZE},
                                                  "MSDU category 0");

  configRegistrar.registerNumeric<std::uint16_t>("msdu1",
                                                  ConfigurationProperties::DEFAULT,
                                                  {MAX_PACKET_SIZE},
                                                  "MSDU category 1");

  configRegistrar.registerNumeric<std::uint16_t>("msdu2",
                                                  ConfigurationProperties::DEFAULT,
                                                  {MAX_PACKET_SIZE},
                                                  "MSDU category 2");

  configRegistrar.registerNumeric<std::uint16_t>("msdu3",
                                                  ConfigurationProperties::DEFAULT,
                                                  {MAX_PACKET_SIZE},
                                                  "MSDU category 3");

  configRegistrar.registerNumeric<std::uint16_t>("cwmin0",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {32},
                                                 "Defines the minimum contention window size in slots"
                                                 " for category 0.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmin1",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {32},
                                                 "Defines the minimum contention window size in slots"
                                                 " for category 1.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmin2",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {16},
                                                 "Defines the minimum contention window size in slots"
                                                 " for category 2.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmin3",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {8},
                                                 "Defines the minimum contention window size in slots"
                                                 " for category 3.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmax0",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {1024},
                                                 "Defines the maximum contention window size in slots"
                                                 " for category 0.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmax1",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {1024},
                                                 "Defines the maximum contention window size in slots"
                                                 " for category 1.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmax2",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {64},
                                                 "Defines the maximum contention window size in slots"
                                                 " for category 2.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<std::uint16_t>("cwmax3",
                                                 ConfigurationProperties::DEFAULT |
                                                 ConfigurationProperties::MODIFIABLE,
                                                 {16},
                                                 "Defines the maximum contention window size in slots"
                                                 " for category 3.",
                                                 1,
                                                 0xffff);

  configRegistrar.registerNumeric<float>("aifs0",
                                         ConfigurationProperties::DEFAULT,
                                         {0.000002f},
                                         "Defines the arbitration inter frame spacing time for category 0"
                                         " and contributes to the calculation of channel access overhead"
                                         " when transmitting category 0 packets.  If WMM is disabled,"
                                         " aifs0 is used for all traffic.",
                                         0.0f,
                                         0.000255f);


  configRegistrar.registerNumeric<float>("aifs1",
                                         ConfigurationProperties::DEFAULT,
                                         {0.000002f},
                                         "Defines the arbitration inter frame spacing time for category 1"
                                         " and contributes to the calculation of channel access overhead"
                                         " when transmitting category 1 packets.",
                                         0.0f,
                                         0.000255);


  configRegistrar.registerNumeric<float>("aifs2",
                                         ConfigurationProperties::DEFAULT,
                                         {0.000002f},
                                         "Defines the arbitration inter frame spacing time for category 2"
                                         " and contributes to the calculation of channel access overhead"
                                         " when transmitting category 2 packets.",
                                         0.0f,
                                         0.000255f);


  configRegistrar.registerNumeric<float>("aifs3",
                                         ConfigurationProperties::DEFAULT,
                                         {0.000001f},
                                         "Defines the arbitration inter frame spacing time for category 3"
                                         " and contributes to the calculation of channel access overhead"
                                         " when transmitting category 3 packets.",
                                         0.0f,
                                         0.000255f);

  configRegistrar.registerNumeric<float>("txop0",
                                         ConfigurationProperties::DEFAULT,
                                         {0.0f},
                                         "Defines the transmit opportunity time for category 0.",
                                         0.0f,
                                         1.0f);

  configRegistrar.registerNumeric<float>("txop1",
                                         ConfigurationProperties::DEFAULT,
                                         {0.0f},
                                         "Defines the transmit opportunity time for category 1.",
                                         0.0f,
                                         1.0f);

  configRegistrar.registerNumeric<float>("txop2",
                                         ConfigurationProperties::DEFAULT,
                                         {0.0f},
                                         "Defines the transmit opportunity time for category 2.",
                                         0.0f,
                                         1.0f);

  configRegistrar.registerNumeric<float>("txop3",
                                         ConfigurationProperties::DEFAULT,
                                         {0.0f},
                                         "Defines the transmit opportunity time for category 3.",
                                         0.0f,
                                         1.0f);

  configRegistrar.registerNumeric<std::uint8_t>("retrylimit0",
                                                ConfigurationProperties::DEFAULT,
                                                {2},
                                                "Defines the maximum number of retries attempted for"
                                                " category 0.");

  configRegistrar.registerNumeric<std::uint8_t>("retrylimit1",
                                                ConfigurationProperties::DEFAULT,
                                                {2},
                                                "Defines the maximum number of retries attempted for"
                                                " category 1.");

  configRegistrar.registerNumeric<std::uint8_t>("retrylimit2",
                                                ConfigurationProperties::DEFAULT,
                                                {2},
                                                "Defines the maximum number of retries attempted for"
                                                " category 2.");

  configRegistrar.registerNumeric<std::uint8_t>("retrylimit3",
                                                ConfigurationProperties::DEFAULT,
                                                {2},
                                                "Defines the maximum number of retries attempted for"
                                                " category 3.");

  configRegistrar.registerNumeric<float>("channelactivityestimationtimer",
                                         ConfigurationProperties::DEFAULT,
                                         {0.1f},
                                         "Defines the channel activity estimation timer in seconds. The"
                                         " timer determines the lag associated with the statistical"
                                         " model used to estimate number of transmitting common and"
                                         " hidden neighbors based on channel activity.",
                                         0.001f,
                                         1.0f);


  configRegistrar.registerNumeric<float>("neighbortimeout",
                                         ConfigurationProperties::DEFAULT,
                                         {30.0f},
                                         "Defines the neighbor timeout in seconds for the neighbor"
                                         " estimation algorithm.",
                                         0.0f,
                                         3600.0f);

  configRegistrar.registerNumeric<bool>("radiometricenable",
                                        ConfigurationProperties::DEFAULT,
                                        {false},
                                        "Defines if radio metrics will be reported up via the Radio to"
                                        " Router Interface (R2RI).");
  


  configRegistrar.registerNumeric<float>("radiometricreportinterval",
                                         ConfigurationProperties::DEFAULT,
                                         {1.0f},
                                         "Defines the  metric report interval in seconds in support of"
                                         " the R2RI feature.",
                                         0.1f,
                                         60.0f);

  configRegistrar.registerNumeric<float>("neighbormetricdeletetime",
                                         ConfigurationProperties::DEFAULT,
                                         {60.0f},
                                         "Defines the time in seconds of no RF receptions from a given"
                                         " neighbor before it is removed from the neighbor table.",
                                         1.0f,
                                         3660.0f);
}


/**
*
* @brief get the promiscuous mode
*
*
* @retval true if on, false if off
*
*/
bool 
EMANE::Models::IEEE80211ABG::MACConfig::getPromiscuosEnable() const
{
  return configItems_.bPromiscousModeEnable_;
}



/**
*
* @brief get the wmm mode
*
* @retval true if on, false if off
*
*/
bool 
EMANE::Models::IEEE80211ABG::MACConfig::getWmmEnable() const
{
  return configItems_.bWmmEnable_;
}


/**
*
* @brief get the modulation type
*
* @retval modulation type
*
*/
EMANE::Models::IEEE80211ABG::MODULATION_TYPE 
EMANE::Models::IEEE80211ABG::MACConfig::getModulationType() const
{
  switch(configItems_.u8ModeIndex_)
    {
      case 3:                    // 80211BG
        return MODULATION_TYPE_80211BG;

      case 2:                    // 80211B
        return MODULATION_TYPE_80211B;

      case 1:                    // 80211A
        return MODULATION_TYPE_80211A;

      default:                   // 80211B
        return MODULATION_TYPE_80211B;
    }
}


/**
*
* @brief get the unicast datarate index
*
* @retval datarate index
*
*/
std::uint8_t 
EMANE::Models::IEEE80211ABG::MACConfig::getUnicastDataRateIndex() const
{
  return configItems_.u8UnicastDataRateIndex_;
}


/**
*
* @brief get the broadcast datarate index
*
* @retval datarate index
*
*/
std::uint8_t 
EMANE::Models::IEEE80211ABG::MACConfig::getBroadcastDataRateIndex() const
{
  return configItems_.u8BroadcastDataRateIndex_;
}


/**
*
* @brief get the unicast datarate
*
* @retval datarate in Kbps
*
*/
std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getUnicastDataRateKbps() const
{
  return UnicastDataRateIndexTable[configItems_.u8UnicastDataRateIndex_];
}


std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getMaxDataRateKbps() const
{
  return UnicastDataRateIndexTable[configItems_.u8UnicastDataRateIndex_] > 
         BroadcastDataRateIndexTable[configItems_.u8BroadcastDataRateIndex_] ?
           UnicastDataRateIndexTable[configItems_.u8UnicastDataRateIndex_] : 
           BroadcastDataRateIndexTable[configItems_.u8BroadcastDataRateIndex_];
}



/**
*
* @brief get the broadcast datarate
*
* @retval datarate in Kbps
*
*/
std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getBroadcastDataRateKbps() const
{
  return BroadcastDataRateIndexTable[configItems_.u8BroadcastDataRateIndex_];
}

/**
*
* @brief get the unicast datarate
*
* @retval datarate in Kbps
*
*/
std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getUnicastDataRateKbps(std::uint8_t u8Index) const
{
  return UnicastDataRateIndexTable[u8Index];
}


/**
*
* @brief get the broadcast datarate
*
* @retval datarate in Kbps
*
*/
std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getBroadcastDataRateKbps(std::uint8_t u8Index) const
{
  return BroadcastDataRateIndexTable[u8Index];
}




/**
*
* @brief get the max ptp distance
*
* @retval max ptp distance
*
*/
std::uint32_t 
EMANE::Models::IEEE80211ABG::MACConfig::getMaxP2pDistance() const
{
  return configItems_.u32MaxP2PDistance_;
}


/**
*
* @brief get the number of access categories (queues)
*
* @retval number of access categories (queues)
*
*/
std::uint8_t 
EMANE::Models::IEEE80211ABG::MACConfig::getNumAccessCategories() const
{
  return configItems_.bWmmEnable_ == true ? MAX_ACCESS_CATEGORIES : 1;
}


/**
*
* @brief get the queue size for a given queue index
*
* @param u8Category queue index
*
* @retval queue size
*
*/
std::uint8_t 
EMANE::Models::IEEE80211ABG::MACConfig::getQueueSize(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.u8QueueSize3_;

      case 2:
        return configItems_.u8QueueSize2_;

      case 1:
        return configItems_.u8QueueSize1_;

      default:
        return configItems_.u8QueueSize0_;
    }
}


/**
*
* @brief get the queue entry size for a given queue index
*
* @param u8Category queue index
*
* @retval queue size
*
*/
std::uint16_t 
EMANE::Models::IEEE80211ABG::MACConfig::getQueueEntrySize(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.u16MSDU3_;

      case 2:
        return configItems_.u16MSDU2_;

      case 1:
        return configItems_.u16MSDU1_;

      default:
        return configItems_.u16MSDU0_;
    }
}



/**
*
* @brief get the min contention window size for a given queue index
*
* @param u8Category queue index
*
* @retval min contention window size
*
*/
std::uint16_t
EMANE::Models::IEEE80211ABG::MACConfig::getCWMin(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.u16CWMin3_;

      case 2:
        return configItems_.u16CWMin2_;

      case 1:
        return configItems_.u16CWMin1_;

      default:
        return configItems_.u16CWMin0_;
    }
}


/**
*
* @brief get the max contention window size for a given queue index
*
* @param u8Category queue index
*
* @retval max contention window size
*
*/
std::uint16_t
EMANE::Models::IEEE80211ABG::MACConfig::getCWMax(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.u16CWMax3_;

      case 2:
        return configItems_.u16CWMax2_;

      case 1:
        return configItems_.u16CWMax1_;

      default:
        return configItems_.u16CWMax0_;
    }
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMin0(std::uint16_t u16Value)
{
  configItems_.u16CWMin0_ = u16Value;

  setCWMinRatioVector(0);
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMin1(std::uint16_t u16Value)
{
  configItems_.u16CWMin1_ = u16Value;

  setCWMinRatioVector(1);
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMin2(std::uint16_t u16Value)
{
  configItems_.u16CWMin2_ = u16Value;

  setCWMinRatioVector(2);
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMin3(std::uint16_t u16Value)
{
  configItems_.u16CWMin3_ = u16Value;

  setCWMinRatioVector(3);
}




void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMax0(std::uint16_t u16Value)
{
  configItems_.u16CWMax0_ = u16Value;
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMax1(std::uint16_t u16Value)
{
  configItems_.u16CWMax1_ = u16Value;
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMax2(std::uint16_t u16Value)
{
  configItems_.u16CWMax2_ = u16Value;
}


void
EMANE::Models::IEEE80211ABG::MACConfig::setCWMax3(std::uint16_t u16Value)
{
  configItems_.u16CWMax3_ = u16Value;
}




/**
*
* @brief get the aifs for a given queue index
*
* @param u8Category queue index
*
* @retval aifs
*
*/
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::MACConfig::getAifsMicroseconds(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.aifsMicroseconds3_;
 
      case 2:
        return configItems_.aifsMicroseconds2_;

      case 1:
        return configItems_.aifsMicroseconds1_;

      default:
        return configItems_.aifsMicroseconds0_;
    }
}



/**
*
* @brief get the txop for a given queue index
*
* @param u8Category queue index
*
* @retval txop
*
*/
EMANE::Microseconds
EMANE::Models::IEEE80211ABG::MACConfig::getTxOpMicroseconds(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.txopMicroseconds3_;
 
      case 2:
        return configItems_.txopMicroseconds2_;

      case 1:
        return configItems_.txopMicroseconds1_;

      default:
        return configItems_.txopMicroseconds0_;
    }
}


/**
*
* @brief get the retry limit for a given queue index
*
* @param u8Category queue index
*
* @retval retry limit
*
*/
std::uint8_t
EMANE::Models::IEEE80211ABG::MACConfig::getRetryLimit(std::uint8_t u8Category) const
{
  switch(u8Category)
    {
      case 3:
        return configItems_.u8RetryLimit3_;
 
      case 2:
        return configItems_.u8RetryLimit2_;

      case 1:
        return configItems_.u8RetryLimit1_;

      default:
        return configItems_.u8RetryLimit0_;
    }
}




/**
*
* @brief get the flow control enable status
*
* @retval flow control enable
*
*/
bool
EMANE::Models::IEEE80211ABG::MACConfig::getFlowControlEnable() const
{
  return configItems_.bFlowControlEnable_;
}



/**
*
* @brief get the number of flow control tokens
*
* @retval flow control tokens
*
*/
std::uint16_t 
EMANE::Models::IEEE80211ABG::MACConfig::getFlowControlTokens() const
{
  return configItems_.u16FlowControlTokens_;
}


/**
*
* @brief get the pcr uri
*
* @retval pcr uri
*
*/
std::string 
EMANE::Models::IEEE80211ABG::MACConfig::getPcrUri() const
{
  return configItems_.sPcrUri_;
}



EMANE::Microseconds 
EMANE::Models::IEEE80211ABG::MACConfig::getNeighborTimeoutMicroseconds() const
{
  return configItems_.neighborTimeoutMicroseconds_;
}



EMANE::Microseconds 
EMANE::Models::IEEE80211ABG::MACConfig::getChannelActivityIntervalMicroseconds() const
{
  return configItems_.channelActivityIntervalMicroseconds_;
}


std::uint16_t 
EMANE::Models::IEEE80211ABG::MACConfig::getRtsThreshold() const
{
  return configItems_.u16RtsThreshold_;
}


EMANE::Models::IEEE80211ABG::CWRatioVector 
EMANE::Models::IEEE80211ABG::MACConfig::getCWMinRatioVector(std::uint8_t u8Category) const
{
  return configItems_.CWMinRatioTable_[u8Category];
}


EMANE::Microseconds 
EMANE::Models::IEEE80211ABG::MACConfig::getNeighborMetricDeleteTimeMicroseconds() const
{
  return configItems_.neighborMetricDeleteTimeMicroseconds_;
}


EMANE::Microseconds 
EMANE::Models::IEEE80211ABG::MACConfig::getRadioMetricReportIntervalMicroseconds() const
{
  return configItems_.radioMetricReportIntervalMicroseconds_;
}


bool
EMANE::Models::IEEE80211ABG::MACConfig::getRadioMetricEnable() const
{
  return configItems_.bRadioMetricEnable_;
}


void 
EMANE::Models::IEEE80211ABG::MACConfig::setCWMinRatioVector(std::uint8_t u8Category)
{
  const std::uint16_t cw{getCWMin(u8Category)};

  for(std::uint8_t u8Index = 0; u8Index < getNumAccessCategories(); ++u8Index)
   {
     const float fRatio{static_cast<float>(cw) / getCWMin(u8Index)};

     configItems_.CWMinRatioTable_[u8Category][u8Index] = fRatio > 1.0f ? 1.0f : fRatio;
   }
}


void 
EMANE::Models::IEEE80211ABG::MACConfig::initCWMinRatioTable()
{
  configItems_.CWMinRatioTable_.resize(getNumAccessCategories());

  for(std::uint8_t u8Category = 0; u8Category < getNumAccessCategories(); ++u8Category)
   {
     configItems_.CWMinRatioTable_[u8Category].resize(getNumAccessCategories());

     setCWMinRatioVector(u8Category);
   }
}

bool 
EMANE::Models::IEEE80211ABG::MACConfig::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(!configureStaticItems(item) && !configureDynamicItems(item))
        {
          LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                  DEBUG_LEVEL,
                                  "MACI %03hu %s::%s invalid config item %s",
                                  id_,
                                  pzLayerName,
                                  __func__,
                                  item.first.c_str());

          return false;
        }
    }

  initCWMinRatioTable();

  return true;
}


bool 
EMANE::Models::IEEE80211ABG::MACConfig::processConfiguration(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(!configureDynamicItems(item))
        {
          return false;
        }
    }

  initCWMinRatioTable();

  return true;
}


bool 
EMANE::Models::IEEE80211ABG::MACConfig::configureStaticItems(const ConfigurationNameAnyValues & item)
{
    if(item.first == "mode")
      {
        configItems_.u8ModeIndex_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8ModeIndex_);
      }
    else if(item.first == "distance")
      {
        configItems_.u32MaxP2PDistance_ = item.second[0].asUINT32();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %u",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u32MaxP2PDistance_);
      }
    else if(item.first == "rtsthreshold")
      {
        configItems_.u16RtsThreshold_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16RtsThreshold_);
      }
    else if(item.first == "wmmenable")
      {
        configItems_.bWmmEnable_ = item.second[0].asBool();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %s",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.bWmmEnable_ ? "on" : "off");
      }
    else if(item.first == "queuesize0")
      {
        configItems_.u8QueueSize0_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                               "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8QueueSize0_);
      }
    else if(item.first == "queuesize1")
      {
        configItems_.u8QueueSize1_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8QueueSize1_);
      }
    else if(item.first == "queuesize2")
      {
        configItems_.u8QueueSize2_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8QueueSize2_);
      }
    else if(item.first == "queuesize3")
      {
        configItems_.u8QueueSize3_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8QueueSize3_);
      }
    else if(item.first == "aifs0")
      {
        configItems_.aifsMicroseconds0_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.aifsMicroseconds0_.count());
      }
    else if(item.first == "aifs1")
      {
        configItems_.aifsMicroseconds1_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.aifsMicroseconds1_.count());
      }
    else if(item.first == "aifs2")
      {
        configItems_.aifsMicroseconds2_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.aifsMicroseconds2_.count());
      }
    else if(item.first == "aifs3")
      {
        configItems_.aifsMicroseconds3_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.aifsMicroseconds3_.count());
      }
    else if(item.first == "msdu0")
      {
        configItems_.u16MSDU0_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16MSDU0_);
      }
    else if(item.first == "msdu1")
      {
        configItems_.u16MSDU1_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16MSDU1_);
      }
    else if(item.first == "msdu2")
      {
        configItems_.u16MSDU2_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                 id_,
                                 pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16MSDU2_);
      }
    else if(item.first == "msdu3")
      {
        configItems_.u16MSDU3_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16MSDU3_);
      }
    else if(item.first == "txop0")
      {
        configItems_.txopMicroseconds0_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.txopMicroseconds0_.count());
      }
    else if(item.first == "txop1")
      {
        configItems_.txopMicroseconds1_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.txopMicroseconds1_.count());
      }
    else if(item.first == "txop2")
      {
        configItems_.txopMicroseconds2_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.txopMicroseconds2_.count());
      }
    else if(item.first == "txop3")
      {
        configItems_.txopMicroseconds3_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{(item.second[0].asFloat())});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %ju",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.txopMicroseconds3_.count());
      }

    else if(item.first == "retrylimit0")
      {
        configItems_.u8RetryLimit0_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8RetryLimit0_);
      }
    else if(item.first == "retrylimit1")
      {
        configItems_.u8RetryLimit1_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8RetryLimit1_);
      }
    else if(item.first == "retrylimit2")
      {
        configItems_.u8RetryLimit2_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8RetryLimit2_);
      }
    else if(item.first == "retrylimit3")
      {
        configItems_.u8RetryLimit3_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8RetryLimit3_);
      }
    else if(item.first == "flowcontrolenable")
      {
        configItems_.bFlowControlEnable_ = item.second[0].asBool();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %s",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.bFlowControlEnable_ ? "on" : "off");
      }
    else if(item.first == "flowcontroltokens")
      {
        configItems_.u16FlowControlTokens_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16FlowControlTokens_);
      }
    else if(item.first == "pcrcurveuri")
      {
        configItems_.sPcrUri_ = item.second[0].asString();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %s",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.sPcrUri_.c_str());
      }
    else if(item.first == "neighbortimeout")
      {
        float fValue{item.second[0].asFloat()};

        configItems_.neighborTimeoutMicroseconds_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{fValue});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %f sec",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                fValue);
      }
    else if(item.first == "channelactivityestimationtimer")
      {
        float fValue{item.second[0].asFloat()};

        configItems_.channelActivityIntervalMicroseconds_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{fValue});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %f sec",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                fValue);
      }
    else if(item.first == "radiometricenable")
      {
        configItems_.bRadioMetricEnable_ = item.second[0].asBool();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %s",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.bRadioMetricEnable_ ? "on" : "off");
      }
    else if(item.first == "radiometricreportinterval")
      {
        float fValue{item.second[0].asFloat()};

        configItems_.radioMetricReportIntervalMicroseconds_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{fValue});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %f sec",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                fValue);
      }
    else if(item.first == "neighbormetricdeletetime")
      {
        float fValue{item.second[0].asFloat()};

        configItems_.neighborMetricDeleteTimeMicroseconds_ = 
          std::chrono::duration_cast<Microseconds>(DoubleSeconds{fValue});

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %f sec",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                fValue);
      }
    else 
      {
         return false;
      }

   return true;
}


bool 
EMANE::Models::IEEE80211ABG::MACConfig::configureDynamicItems(const ConfigurationNameAnyValues & item)
{
    if(item.first == "unicastrate")
      {
        configItems_.u8UnicastDataRateIndex_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8UnicastDataRateIndex_);
      }
    else if(item.first == "multicastrate")
      {
        configItems_.u8BroadcastDataRateIndex_ = item.second[0].asUINT8();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hhu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u8BroadcastDataRateIndex_);
      }
    else if(item.first == "cwmin0")
      {
        configItems_.u16CWMin0_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMin0_);
      }
    else if(item.first == "cwmin1")
      {
        configItems_.u16CWMin1_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMin1_);
      }
    else if(item.first == "cwmin2")
      {
        configItems_.u16CWMin2_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMin2_);
      }
    else if(item.first == "cwmin3")
      {
        configItems_.u16CWMin3_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMin3_);
      }
    else if(item.first == "cwmax0")
      {
        configItems_.u16CWMax0_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMax0_);
      }
    else if(item.first == "cwmax1")
      {
        configItems_.u16CWMax1_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMax1_);
      }
    else if(item.first == "cwmax2")
      {
        configItems_.u16CWMax2_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMax2_);
      }
    else if(item.first == "cwmax3")
      {
        configItems_.u16CWMax3_ = item.second[0].asUINT16();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %hu",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.u16CWMax3_);
      }
    else if(item.first == "enablepromiscuousmode")
      {
        configItems_.bPromiscousModeEnable_ = item.second[0].asBool();

        LOGGER_STANDARD_LOGGING(logServiceProvider_,
                                INFO_LEVEL,
                                "MACI %03hu %s::%s %s = %s",
                                id_,
                                pzLayerName,
                                __func__,
                                item.first.c_str(),
                                configItems_.bPromiscousModeEnable_ ? "on" : "off");
      }
    else
      {
         return false;
      }

   return true;
}
