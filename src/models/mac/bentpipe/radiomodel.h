/*
 * Copyright (c) 2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_RADIOMODEL_HEADER_
#define EMANE_MODELS_BENTPIPE_RADIOMODEL_HEADER_

#include "antennas.h"
#include "transponder.h"
#include "transponderuser.h"
#include "transponderpackettransport.h"
#include "transponderconfiguration.h"
#include "receivemanager.h"
#include "queuemanager.h"
#include "pcrmanager.h"
#include "packetstatuspublisher.h"
#include "antennastatuspublisher.h"
#include "transponderstatuspublisher.h"
#include "neighborstatuspublisher.h"
#include "slotstatuspublisher.h"
#include "configurationvalidationmanager.h"

#include "emane/maclayerimpl.h"
#include "emane/utils/randomnumberdistribution.h"

#include <memory>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      class RadioModel : public MACLayerImplementor,
                         public TransponderUser,
                         public TransponderPacketTransport
      {
      public:
        RadioModel(NEMId id,
                   PlatformServiceProvider *pPlatformServiceProvider,
                   RadioServiceProvider * pRadioServiceProvider);

        ~RadioModel();

        void initialize(Registrar & registrar) override;

        void configure(const ConfigurationUpdate & update) override;

        void start() override;

        void postStart() override;

        void stop() override;

        void destroy() noexcept override;

        void processUpstreamControl(const ControlMessages & msgs) override;


        void processUpstreamPacket(const CommonMACHeader & hdr,
                                   UpstreamPacket & pkt,
                                   const ControlMessages & msgs) override;

        void processDownstreamControl(const ControlMessages & msgs) override;


        void processDownstreamPacket(DownstreamPacket & pkt,
                                     const ControlMessages & msgs) override;


        void processConfiguration(const ConfigurationUpdate & update) override;

        void notifyTxOpportunity(Transponder *pTransponder) override;

        void ubendPacket(DownstreamPacket & pkt,
                         TransponderIndex tansponderIxdex) override;


        void processPacket(UpstreamPacket & pkt) override;

      private:
        using Transponders = std::map<TransponderIndex,std::unique_ptr<Transponder>>;
        Transponders transponders_;

        Antennas antennas_;
        using AntennaIndexSet = std::set<AntennaIndex>;
        AntennaIndexSet registeredRxAntenna_;

        using TransponderIndexSet = std::set<TransponderIndex>;

        using AntennaTransponderMap =
          std::map<std::pair<AntennaIndex,
                             std::uint64_t>,
                   TransponderIndex>;

        using AnntennaFrequencyMap = std::map<AntennaIndex,FrequencySet>;
        AnntennaFrequencyMap antennaFrequencyMap_{};

        AntennaTransponderMap antennaTransponderMap_;

        PacketStatusPublisher packetStatusPublisher_;
        using ReceiveManagers =
          std::map<TransponderIndex,std::unique_ptr<ReceiveManager>>;
        ReceiveManagers receiveManagers_;
        QueueManager queueManager_;
        std::array<int,256> tosToTransponder_;
        PCRManager pcrManager_;

        std::uint64_t u64SequenceNumber_;

        AntennaStatusPublisher antennaStatusPublisher_;
        TransponderStatusPublisher transponderStatusPublisher_;
        NeighborStatusPublisher neighborStatusPublisher_;
        SlotStatusPublisher slotStatusPublisher_;

        ConfigurationValidationManager configurationValidationManager_;

        void loadConfiguration(const ConfigurationUpdate & update);

        void process(const TimePoint & now);
      };
    }
  }
}


#endif // EMANE_MODELS_BENTPIPE_RADIOMODEL_HEADER_
