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

#include "nemqueuedlayer.h"
#include "logservice.h"

#include <exception>
#include <mutex>

EMANE::NEMQueuedLayer::NEMQueuedLayer(NEMId id, PlatformServiceProvider *pPlatformService):
  NEMLayer{id, pPlatformService},
  pPlatformService_{pPlatformService},
  thread_{},
  bCancel_{},
  pProcessedDownstreamPacket_{},
  pProcessedUpstreamPacket_{},
  pProcessedDownstreamControl_{},
  pProcessedUpstreamControl_{},
  pProcessedEvent_{},
  pProcessedTimedEvent_{},
  pProcessedConfiguration_{}{}

EMANE::NEMQueuedLayer::~NEMQueuedLayer()
{
  std::lock_guard<std::mutex> m(mutex_);

  if(!bCancel_ && thread_.joinable())
    {
      bCancel_ = true;
      cond_.notify_one();
    }
}

void  EMANE::NEMQueuedLayer::initialize(Registrar & registrar)
{
  auto & statisticRegistrar = registrar.statisticRegistrar();

  pProcessedDownstreamPacket_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedDownstreamPackets",
                                                      StatisticProperties::CLEARABLE);

  pProcessedUpstreamPacket_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedUpstreamPackets",
                                                      StatisticProperties::CLEARABLE);

  pProcessedDownstreamControl_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedDownstreamControl",
                                                      StatisticProperties::CLEARABLE);

  pProcessedUpstreamControl_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedUpstreamControl",
                                                      StatisticProperties::CLEARABLE);
  pProcessedEvent_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedEvents",
                                                      StatisticProperties::CLEARABLE);
  pProcessedConfiguration_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedConfiguration",
                                                      StatisticProperties::CLEARABLE);
  pProcessedTimedEvent_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedTimedEvents",
                                                      StatisticProperties::CLEARABLE);

  pStatisticHistogramTable_.reset(new Utils::StatisticHistogramTable<EventId>{
      statisticRegistrar,
        "EventReceptionTable",
          {"Event","Total Rx"},
        "Received event counts"});

  avgQueueWait_.registerStatistic
    (statisticRegistrar.registerNumeric<double>("avgProcessAPIQueueWait",
                                                StatisticProperties::CLEARABLE,
                                                "Average API queue wait for a processUpstreamPacket,"
                                                " processUpstreamControl, processDownstreamPacket,"
                                                " processDownstreamControl, processEvent and"
                                                " processTimedEvent in microseconds."));

  avgQueueDepth_.registerStatistic
    (statisticRegistrar.registerNumeric<double>("avgProcessAPIQueueDepth",
                                               StatisticProperties::CLEARABLE,
                                               "Average API queue depth for a processUpstreamPacket,"
                                               " processUpstreamControl, processDownstreamPacket,"
                                               " processDownstreamControl, processEvent and"
                                               " processTimedEvent."));

  avgTimedEventLatency_.registerStatistic
    (statisticRegistrar.registerNumeric<double>("avgTimedEventLatency",
                                                StatisticProperties::CLEARABLE));

  avgTimedEventLatencyRatio_.registerStatistic
    (statisticRegistrar.registerNumeric<double>("avgTimedEventLatencyRatio",
                                                StatisticProperties::CLEARABLE,
                                                "Average ratio of the delta between the scheduled timer"
                                                " expiration and the actual firing over the requested"
                                                " duration. An average ratio approaching 1 indicates that"
                                                " timer latencies are large in comparison to the requested"
                                                " durations."));
}

void EMANE::NEMQueuedLayer::start()
{
  thread_ = std::thread{&EMANE::NEMQueuedLayer::processWorkQueue,this};
}

void EMANE::NEMQueuedLayer::stop()
{
  mutex_.lock();
  bCancel_ = true;
  cond_.notify_one();
  mutex_.unlock();
  thread_.join();
  bCancel_ = false;
}

void EMANE::NEMQueuedLayer::processConfiguration(const ConfigurationUpdate & update)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessConfiguration,this,Clock::now(),update));
  avgQueueDepth_.update(queue_.size());
  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processDownstreamControl(const ControlMessages & msgs)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessDownstreamControl,this,Clock::now(),msgs));
  avgQueueDepth_.update(queue_.size());
  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processDownstreamPacket(DownstreamPacket & pkt,const ControlMessages & msgs)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessDownstreamPacket,this,Clock::now(),pkt,msgs));
  avgQueueDepth_.update(queue_.size());
  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processUpstreamPacket(UpstreamPacket & pkt,const ControlMessages & msgs)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessUpstreamPacket,this,Clock::now(),pkt,msgs));
  avgQueueDepth_.update(queue_.size());
  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processUpstreamControl(const ControlMessages & msgs)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessUpstreamControl,this,Clock::now(),msgs));
  avgQueueDepth_.update(queue_.size());
  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processEvent(const EventId & eventId,
                                         const Serialization & serialization)
{
  std::lock_guard<std::mutex> m(mutex_);

  queue_.push(std::bind(&NEMQueuedLayer::handleProcessEvent,
                        this,
                        Clock::now(),
                        eventId,
                        serialization));

  avgQueueDepth_.update(queue_.size());

  cond_.notify_one();
}

void EMANE::NEMQueuedLayer::processTimedEvent(TimerEventId eventId,
                                              const TimePoint & expireTime,
                                              const TimePoint & scheduleTime,
                                              const TimePoint & fireTime,
                                              const void * arg)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push(std::bind(&NEMQueuedLayer::handleProcessTimedEvent,
                        this,
                        Clock::now(),
                        eventId,
                        expireTime,
                        scheduleTime,
                        fireTime,
                        arg));

  avgQueueDepth_.update(queue_.size());

  cond_.notify_one();
}


void EMANE::NEMQueuedLayer::processWorkQueue()
{
  while(1)
    {
      std::unique_lock<std::mutex> lock{mutex_};

      while(queue_.empty() && !bCancel_)
        {
          cond_.wait(lock);
        }

      if(bCancel_)
        {
          break;
        }

      // retrieve the next funtion to execute
      auto f = queue_.front();

      // remove the functor from the top of the queue
      queue_.pop();

      // release the queue syncronization object
      lock.unlock();

      try
        {
          // execute the funtor
          f();
        }
      catch(std::exception & exp)
        {
          // cannot really do too much at this point, so we'll log it
          // good candidate spot to generate and error event as well
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                 "%03hu NEMQueuedLayer::processWorkQueue:"
                                 " Exception caught %s",
                                 id_,
                                 exp.what());
        }
      catch(...)
        {
          // cannot really do too much at this point, so we'll log it
          // good candidate spot to generate and error event as well
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "%03hu NEMQueuedLayer::processWorkQueue:"
                                  " Exception caught",
                                  id_);
        }
    }
}

  void EMANE::NEMQueuedLayer::handleProcessConfiguration(TimePoint enqueueTime,
                                                         const ConfigurationUpdate update)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedConfiguration_;

  doProcessConfiguration(update);
}

    void EMANE::NEMQueuedLayer::handleProcessDownstreamControl(TimePoint enqueueTime,
                                                               const ControlMessages msgs)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedDownstreamControl_;

  doProcessDownstreamControl(msgs);

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::NEMQueuedLayer::handleProcessDownstreamPacket(TimePoint enqueueTime,
                                                          DownstreamPacket pkt,
                                                          const ControlMessages msgs)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedDownstreamPacket_;

  doProcessDownstreamPacket(pkt,msgs);

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

  void EMANE::NEMQueuedLayer::handleProcessUpstreamPacket(TimePoint enqueueTime,
                                                          UpstreamPacket pkt,
                                                          const ControlMessages msgs)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedUpstreamPacket_;

  doProcessUpstreamPacket(pkt,msgs);

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::NEMQueuedLayer::handleProcessUpstreamControl(TimePoint enqueueTime,
                                                         const ControlMessages msgs)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedUpstreamControl_;

  doProcessUpstreamControl(msgs);

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

  void EMANE::NEMQueuedLayer::handleProcessEvent(TimePoint enqueueTime,
                                                 const EventId eventId,
                                                 const Serialization serialization)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  pStatisticHistogramTable_->increment(eventId);

  ++*pProcessedEvent_;

  doProcessEvent(eventId,serialization);
}

void EMANE::NEMQueuedLayer::handleProcessTimedEvent(TimePoint enqueueTime,
                                                    TimerEventId eventId,
                                                    const TimePoint & expireTime,
                                                    const TimePoint & scheduleTime,
                                                    const TimePoint & fireTime,
                                                    const void * arg)

{
  auto now = Clock::now();

  avgTimedEventLatency_.update(std::chrono::duration_cast<Microseconds>(now - expireTime).count());

  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(now - enqueueTime).count());

  auto duration = std::chrono::duration_cast<Microseconds>(expireTime - scheduleTime);

  if(duration.count() > 0)
    {
      avgTimedEventLatencyRatio_.update(std::chrono::duration_cast<Microseconds>(fireTime - expireTime).count() /
                                        static_cast<double>(duration.count()));
    }
  else
    {
      avgTimedEventLatencyRatio_.update(1);
    }

  ++*pProcessedTimedEvent_;

  doProcessTimedEvent(eventId,expireTime,scheduleTime,fireTime,arg);
}
