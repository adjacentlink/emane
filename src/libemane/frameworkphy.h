/*
 * Copyright (c) 2013-2014,2016-2017 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANEFRAMEWORK_HEADER_
#define EMANEFRAMEWORK_HEADER_

#include "emane/phylayerimpl.h"
#include "emane/phytypes.h"
#include "emane/utils/commonlayerstatistics.h"

#include "emane/models/frameworkphy/locationmanager.h"
#include "emane/models/frameworkphy/spectrummonitor.h"
#include "emane/models/frameworkphy/gainmanager.h"
#include "emane/models/frameworkphy/propagationmodelalgorithm.h"
#include "emane/models/frameworkphy/eventtablepublisher.h"
#include "emane/models/frameworkphy/receivepowertablepublisher.h"
#include "emane/models/frameworkphy/fadingmanager.h"

#include <set>
#include <cstdint>
#include <memory>

namespace EMANE
{
  class FrameworkPHY : public PHYLayerImplementor
  {
  public:
    FrameworkPHY(NEMId id,
                 PlatformServiceProvider* pPlatformService,
                 RadioServiceProvider * pRadioService);

    ~FrameworkPHY();

    void initialize(Registrar & registrar) override;

    void configure(const ConfigurationUpdate & update) override;

    void start() override;

    void stop() override;

    void destroy() throw() override;

    void processConfiguration(const ConfigurationUpdate & update) override;

    void processUpstreamPacket(const CommonPHYHeader & hdr,
                               UpstreamPacket & pkt,
                               const ControlMessages & msgs) override;

    // provides test harness access
    void processUpstreamPacket_i(const TimePoint & now,
                                 const CommonPHYHeader & hdr,
                                 UpstreamPacket & pkt,
                                 const ControlMessages & msgs);

    void processDownstreamControl(const ControlMessages & msgs) override;

    void processDownstreamPacket(DownstreamPacket & pkt,
                                 const ControlMessages & msgs) override;

    void processEvent(const EventId & eventId,
                      const Serialization & serialization) override;


    SpectrumMonitor & getSpectrumMonitor();

  private:
    SpectrumMonitor * pSpectrumMonitor_;
    GainManager gainManager_;
    LocationManager locationManager_;
    std::uint64_t u64BandwidthHz_;
    double dTxPowerdBm_;
    std::uint64_t u64TxFrequencyHz_;
    double dReceiverSensitivitydBm_;
    SpectrumMonitor::NoiseMode noiseMode_;
    std::uint16_t u16SubId_;
    std::uint16_t u16TxSequenceNumber_;
    std::pair<double,bool> optionalFixedAntennaGaindBi_;
    std::unique_ptr<PropagationModelAlgorithm> pPropagationModelAlgorithm_;
    Utils::CommonLayerStatistics commonLayerStatistics_;
    EventTablePublisher eventTablePublisher_;
    ReceivePowerTablePublisher receivePowerTablePublisher_;
    Microseconds noiseBinSize_;
    Microseconds maxSegmentOffset_;
    Microseconds maxMessagePropagation_;
    Microseconds maxSegmentDuration_;
    Microseconds timeSyncThreshold_;
    bool bNoiseMaxClamp_;
    double dSystemNoiseFiguredB_;
    StatisticNumeric<std::uint64_t> * pTimeSyncThresholdRewrite_;
    FadingManager fadingManager_;
  };
}

#endif // EMANEFRAMEWORK_HEADER_

/**
 * @page EmulatorPhysicalLayer Emulator Physical Layer
 *
 * @section RadioModelInterfacing Radio Model Interfacing
 *
 * A radio model interfaces with the @ref EMANE::FrameworkPHY "emulator physical layer" using @ref
 * EMANE::ControlMessage "ControlMessages".
 *
 * @subsection FrequencyControlMessage FrequencyControlMessage
 *
 * The @ref EMANE::Controls::FrequencyControlMessage "Controls::FrequencyControlMessage" is used in
 * both upstream and downstream packet processing and only valid when received as an argument to @ref
 * EMANE::UpstreamTransport::processUpstreamPacket "UpstreamTransport::processUpstreamPacket" or
 * @ref EMANE::DownstreamTransport::processDownstreamPacket
 * "DownstreamTransport::processDownstreamPacket".
 *
 * In the downstream direction, a @ref EMANE::MACLayerImplementor "radio model" must send a
 * @ref EMANE::Controls::FrequencyControlMessage "FrequencyControlMessage" with
 * every @ref EMANE::DownstreamPacket "DownstreamPacket". This control message is used to specify
 * one or more @ref EMANE::FrequencySegment "FrequencySegments" to use during message transmission and
 * to optionally set the transmitter bandwidth.
 *
 * Radio model send downstream packet example with a @ref EMANE::Controls::FrequencyControlMessage
 * "FrequencyControlMessage:
 * @snippet src/models/mac/rfpipe/maclayer.cc pysicallayer-frequencycontrolmessage-snippet
 *
 * For a @ref EMANE::Controls::FrequencyControlMessage "FrequencyControlMessage" to be valid it must
 * contain at least one @ref EMANE::FrequencySegment "FrequencySegment". A @ref EMANE::FrequencySegment
 * "FrequencySegment" contains the segment center frequency, the segment offset from the @ref
 * EMANE::CommonPHYHeader "CommonPHYHeader" transmission time and the segment duration. A special
 * frequency value of 0 Hz will cause the physical layer to use its configured frequency. Likewise,
 * a @ref EMANE::Controls::FrequencyControlMessage "FrequencyControlMessage" with a bandwidth of 0 Hz
 * will cause the physical layer's configured bandwidth to be used.
 *
 * In the upstream direction, the physical layer will send a @ref EMANE::Controls::FrequencyControlMessage
 *  "FrequencyControlMessage" that contains each received @ref EMANE::FrequencySegment "FrequencySegment"
 *  along with the segment's receive power in dBm with every @ref EMANE::UpstreamPacket "UpstreamPacket".
 *
 * Physical Layer send upstream packet example with a @ref EMANE::Controls::FrequencyControlMessage
 * "FrequencyControlMessage:
 * @snippet src/libemane/frameworkphy.cc physicallayer-sendupstreampacket-snippet
 *
 * The @ref EMANE::SpectrumMonitor "SpectrumMonitor" will remove @ref EMANE::FrequencySegment
 * "FrequencySegments" that are below receiver sensitivity. Provided there is at least one
 * @ref EMANE::FrequencySegment "FrequencySegment" remaining, an @ref
 * EMANE::UpstreamPacket "UpstreamPacket" will be sent to the radio model for processing.
 *
 * An OTA message that matches physical layer registration id and physical layer
 * subid and contains at least one @ref EMANE::FrequencySegment "FrequencySegment" that is not in the
 * frequency of interest list, will cause the entire message to be treated as out-of-band noise.
 *
 * @subsection TransmitterControlMessage TransmitterControlMessage
 *
 * The @ref EMANE::Controls::TransmitterControlMessage "Controls::TransmitterControlMessage" is used in
 * downstream packet processing and only valid when received as an argument to @ref
 * EMANE::DownstreamTransport::processDownstreamPacket "DownstreamTransport::processDownstreamPacket".
 *
 * A radio model can send a @ref EMANE::Controls::TransmitterControlMessage "TransmitterControlMessage"
 * to indicate more than one transmitter is sending the OTA message as part of a collaborative transmission
 * (constructive interference). This will result in a single OTA message with multiple transmitters.
 *
 * When the physical layer receives an OTA collaborative transmission it will sum up the receive powers for each
 * frequency segment before sending the message to the radio model for processing.
 *
 * Radio model send downstream packet example with a @ref EMANE::Controls::TransmitterControlMessage
 * "TransmitterControlMessage":
 * @snippet src/models/shim/phyapitest/shimlayer.cc physicallayer-transmittercontrolmessage-snippet
 *
 * @subsection TimeStampControlMessage TimeStampControlMessage
 *
 * The @ref EMANE::Controls::TimeStampControlMessage "Controls::TimeStampControlMessage" is used in
 * downstream packet processing and only valid when received as an argument to @ref
 * EMANE::DownstreamTransport::processDownstreamPacket "DownstreamTransport::processDownstreamPacket".
 *
 * A radio model can send a @ref EMANE::Controls::TimeStampControlMessage "TimeStampControlMessage"
 * to specify the transmission time stamp used in the @ref EMANE::CommonPHYHeader "CommonPHYHeader". This
 * time should be the start-of-transmission time for the message.
 *
 * Radio model send downstream packet example with a @ref EMANE::Controls::TimeStampControlMessage
 * "TimeStampControlMessage":
 * @snippet src/models/mac/rfpipe/maclayer.cc pysicallayer-frequencycontrolmessage-snippet
 *
 * If this control message is not present the physical layer will use the current time as the
 * transmission time stamp.
 *
 * @subsection ReceivePropertiesControlMessage ReceivePropertiesControlMessage
 *
 * The @ref EMANE::Controls::ReceivePropertiesControlMessage "Controls::ReceivePropertiesControlMessage"
 * is used in upstream packet processing and only valid when received as an argument to @ref
 * EMANE::UpstreamTransport::processUpstreamPacket "UpstreamTransport::processUpstreamPacket".
 *
 * The physical layer will send a @ref EMANE::Controls::ReceivePropertiesControlMessage
 * "ReceivePropertiesControlMessage" with every @ref EMANE::UpstreamPacket "UpstreamPacket" packet.
 * This control message contains the message transmission time, propagation delay, span and receiver
 * sensitivity. See @ref SpectrumWindow for more information.
 *
 * Physical Layer send upstream packet example with a @ref EMANE::Controls::ReceivePropertiesControlMessage
 * "Controls::ReceivePropertiesControlMessage":
 * @snippet src/libemane/frameworkphy.cc physicallayer-sendupstreampacket-snippet
 *
 *  @subsection AntennaProfileControlMessage AntennaProfileControlMessage
 *
 * The @ref EMANE::Controls::AntennaProfileControlMessage "Controls::AntennaProfileControlMessage" is used in
 * downstream processing and only valid when received as an argument to @ref
 * EMANE::DownstreamTransport::processDownstreamPacket "DownstreamTransport::processDownstreamPacket" or
 * @ref EMANE::DownstreamTransport::processDownstreamControl "DownstreamTransport::processDownstreamControl.
 *
 *
 * A radio model can send an @ref EMANE::Controls::AntennaProfileControlMessage "AntennaProfileControlMessage"
 * to change the current antenna profile id and antenna pointing information. All physical layer instances must
 * be aware of each others antenna profile id and antenna pointing information.
 *
 * If the physical layer receives an @ref EMANE::Controls::AntennaProfileControlMessage "AntennaProfileControlMessage"
 * as an argument to @ref EMANE::DownstreamTransport::processDownstreamControl "processDownstreamControl", it will send
 * an @ref EMANE::Events::AntennaProfileEvent "Events::AntennaProfileEvent" to inform all NEMs of the change.
 *
 * @snippet src/libemane/frameworkphy.cc eventservice-sendevent-snippet
 *
 * If the physical layer receives an @ref EMANE::Controls::AntennaProfileControlMessage "AntennaProfileControlMessage"
 * as an argument to @ref EMANE::DownstreamTransport::processDownstreamPacket "processDownstreamPacket", it will attach
 * an @ref EMANE::Events::AntennaProfileEvent "AntennaProfileEvent" to the packet using @ref
 * EMANE::DownstreamPacket::attachEvent "DownstreamPacket::attachEvent". This @a attached event will be processed by
 * all other physical layers as if it were sent over the event channel with the additional guarantee that it will be processed
 * before the packet is processed.
 *
 * @snippet src/libemane/frameworkphy.cc physicallayer-attachevent-snippet
 */
