/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
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

#ifndef EMANENEMQUEUEDLAYER_HEADER_
#define EMANENEMQUEUEDLAYER_HEADER_

#include "emane/nemlayer.h"

#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "emane/utils/runningaverage.h"
#include "emane/utils/statistichistogramtable.h"

namespace EMANE
{
  /**
   * @class NEMQueuedLayer
   *
   * @brief A layer stack with a porcessing queue between
   * each layer to decouple to intra queue processing
   *
   * @note Transport processing is deferred using function objects.  A
   * processing thread is then used to work the function object queue
   * processing packets, control, and events in a thread safe sequential
   * manner.
   */
  class NEMQueuedLayer :  public NEMLayer
  {
  public:
    ~NEMQueuedLayer();

    void initialize(Registrar & registrar) override;

    void start() override;

    void stop() override;

    void processConfiguration(const ConfigurationUpdate & update) override;

    void processDownstreamControl(const ControlMessages & msgs) override;

    void processDownstreamPacket(DownstreamPacket &pkt, const ControlMessages & msgs) override;

    void processUpstreamPacket(UpstreamPacket &pkt, const ControlMessages & msgs) override;

    void processUpstreamControl(const ControlMessages & msgs) override;

    void processEvent(const EventId &eventId, const Serialization &serialization) override;

    void processTimedEvent(TimerEventId eventId,
                           const TimePoint & expireTime,
                           const TimePoint & scheduleTime,
                           const TimePoint & fireTime,
                           const void * arg) override;

  protected:
    NEMQueuedLayer(NEMId id, PlatformServiceProvider * pPlatformService);

    virtual void doProcessConfiguration(const ConfigurationUpdate &) = 0;

    virtual void doProcessDownstreamControl(const ControlMessages &) = 0;

    virtual void doProcessDownstreamPacket(DownstreamPacket &, const ControlMessages &) = 0;

    virtual void doProcessUpstreamPacket(UpstreamPacket &, const ControlMessages &) = 0;

    virtual void doProcessUpstreamControl(const ControlMessages &) = 0;

    virtual void doProcessEvent(const EventId &, const Serialization &) = 0;

    virtual void doProcessTimedEvent(TimerEventId eventId,
                                     const TimePoint & expireTime,
                                     const TimePoint & scheduleTime,
                                     const TimePoint & fireTime,
                                     const void * arg) = 0;

  private:
    using MessageProcessingQueue = std::queue<std::function<void()>>;
    PlatformServiceProvider * pPlatformService_;
    std::thread thread_;
    MessageProcessingQueue queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool bCancel_;

    StatisticNumeric<std::uint64_t> * pProcessedDownstreamPacket_;
    StatisticNumeric<std::uint64_t> * pProcessedUpstreamPacket_;
    StatisticNumeric<std::uint64_t> * pProcessedDownstreamControl_;
    StatisticNumeric<std::uint64_t> * pProcessedUpstreamControl_;
    StatisticNumeric<std::uint64_t> * pProcessedEvent_;
    StatisticNumeric<std::uint64_t> * pProcessedTimedEvent_;
    StatisticNumeric<std::uint64_t> * pProcessedConfiguration_;

    Utils::RunningAverage<double> avgQueueWait_;
    Utils::RunningAverage<double> avgQueueDepth_;
    Utils::RunningAverage<double> avgTimedEventLatency_;
    Utils::RunningAverage<double> avgTimedEventLatencyRatio_;

    std::unique_ptr<Utils::StatisticHistogramTable<EventId>> pStatisticHistogramTable_;

    NEMQueuedLayer(const NEMQueuedLayer &);

    void processWorkQueue();

    void handleProcessConfiguration(TimePoint enqueueTime,
                                    const ConfigurationUpdate);

    void handleProcessDownstreamControl(TimePoint enqueueTime,
                                        const ControlMessages);

    void handleProcessDownstreamPacket(TimePoint enqueueTime,
                                       DownstreamPacket,
                                       const ControlMessages);

    void handleProcessUpstreamPacket(TimePoint enqueueTime,
                                     UpstreamPacket,
                                     const ControlMessages);

    void handleProcessUpstreamControl(TimePoint enqueueTime,
                                      const ControlMessages);

    void handleProcessEvent(TimePoint enqueueTime,
                            const EventId,
                            const Serialization);

    void handleProcessTimedEvent(TimePoint enqueueTime,
                                 TimerEventId eventId,
                                 const TimePoint & expireTime,
                                 const TimePoint & scheduleTime,
                                 const TimePoint & fireTime,
                                 const void * arg);

  };
}

#endif //EMANENEMQUEUEDLAYER_HEADER_
