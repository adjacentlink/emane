/*
 * Copyright (c) 2013-2014,2016-2017 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <cstring>

namespace
{
  const uint64_t one{1};
}

EMANE::NEMQueuedLayer::NEMQueuedLayer(NEMId id, PlatformServiceProvider *pPlatformService):
  NEMLayer{id, pPlatformService},
  pPlatformService_{pPlatformService},
  thread_{},
  iFd_{},
  iepollFd_{},
  bCancel_{},
  pProcessedDownstreamPacket_{},
  pProcessedUpstreamPacket_{},
  pProcessedDownstreamControl_{},
  pProcessedUpstreamControl_{},
  pProcessedEvent_{},
  pProcessedTimedEvent_{},
  pProcessedConfiguration_{}
{
  iFd_ = eventfd(0,0);

  iepollFd_ = epoll_create1(0);

  // add the eventfd socket to the epoll instance
  struct epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = iFd_;

  if(epoll_ctl(iepollFd_,EPOLL_CTL_ADD,iFd_,&ev) == -1)
    {
      // cannot really do too much at this point, so we'll log it
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,
                              "%03hu NEMQueuedLayer::NEMQueuedLayer:"
                              " unable to add eventfd to epoll",
                              id_);
    }
}

EMANE::NEMQueuedLayer::~NEMQueuedLayer()
{
  mutex_.lock();

  if(!bCancel_ && thread_.joinable())
    {
      bCancel_ = true;
      write(iFd_,&one,sizeof(one));
      mutex_.unlock();
      thread_.join();
      mutex_.lock();
    }

  mutex_.unlock();

  close(iFd_);
}

void  EMANE::NEMQueuedLayer::initialize(Registrar & registrar)
{
  auto & statisticRegistrar = registrar.statisticRegistrar();

  pProcessedDownstreamPacket_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedDownstreamPackets",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed downstream packets.");

  pProcessedUpstreamPacket_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedUpstreamPackets",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed upstream packets.");

  pProcessedDownstreamControl_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedDownstreamControl",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed downstream control.");

  pProcessedUpstreamControl_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedUpstreamControl",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed upstream control.");
  pProcessedEvent_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedEvents",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed events.");
  pProcessedConfiguration_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedConfiguration",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed configuration.");
  pProcessedTimedEvent_ =
    statisticRegistrar.registerNumeric<std::uint64_t>("processedTimedEvents",
                                                      StatisticProperties::CLEARABLE,
                                                      "The number of processed timed events.");

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
                                                StatisticProperties::CLEARABLE,
                                                "Average latency between the scheduled timer"
                                                " expiration and the actual firing over the requested"
                                                " duration."));

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
  write(iFd_,&one,sizeof(one));
  mutex_.unlock();
  thread_.join();
  bCancel_ = false;
}

void EMANE::NEMQueuedLayer::enqueue_i(QCallback && callback)
{
  std::lock_guard<std::mutex> m(mutex_);
  queue_.push_back(std::move(callback));
  avgQueueDepth_.update(queue_.size());
  write(iFd_,&one,sizeof(one));
}

void EMANE::NEMQueuedLayer::processConfiguration(const ConfigurationUpdate & update)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessConfiguration,
                      this,
                      Clock::now(),
                      update));
}

void EMANE::NEMQueuedLayer::processDownstreamControl(const ControlMessages & msgs)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessDownstreamControl,
                      this,
                      Clock::now(),
                      msgs));
}

void EMANE::NEMQueuedLayer::processDownstreamPacket(DownstreamPacket & pkt,
                                                    const ControlMessages & msgs)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessDownstreamPacket,
                      this,
                      Clock::now(),
                      pkt,
                      msgs));

}

void EMANE::NEMQueuedLayer::processUpstreamPacket(UpstreamPacket & pkt,const ControlMessages & msgs)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessUpstreamPacket,
                      this,
                      Clock::now(),
                      pkt,
                      msgs));
}

void EMANE::NEMQueuedLayer::processUpstreamControl(const ControlMessages & msgs)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessUpstreamControl,
                      this,
                      Clock::now(),
                      msgs));
}

void EMANE::NEMQueuedLayer::processEvent(const EventId & eventId,
                                         const Serialization & serialization)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessEvent,
                      this,
                      Clock::now(),
                      eventId,
                      serialization));

}

void EMANE::NEMQueuedLayer::processTimedEvent(TimerEventId eventId,
                                              const TimePoint & expireTime,
                                              const TimePoint & scheduleTime,
                                              const TimePoint & fireTime,
                                              const void * arg)
{
  enqueue_i(std::bind(&NEMQueuedLayer::handleProcessTimedEvent,
                      this,
                      Clock::now(),
                      eventId,
                      expireTime,
                      scheduleTime,
                      fireTime,
                      arg));
}

void EMANE::NEMQueuedLayer::processWorkQueue()
{
  std::uint64_t u64Expired{};
#define MAX_EVENTS 32
  struct epoll_event events[MAX_EVENTS];
  int nfds{};

  while(!bCancel_)
    {
      nfds = epoll_wait(iepollFd_,events,MAX_EVENTS,-1);

      if(nfds == -1)
        {
          if(errno == EINTR)
            {
              continue;
            }

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "%03hu NEMQueuedLayer::processWorkQueue:"
                                  " epoll_wait error: %s",
                                  id_,
                                  strerror(errno));
          break;
        }

      for(int n = 0; n < nfds; ++n)
        {
          if(events[n].data.fd == iFd_)
            {
              // wait for an interval timer to expire
              if(read(iFd_,&u64Expired,sizeof(u64Expired)) > 0)
                {
                  std::unique_lock<std::mutex> lock(mutex_);

                  if(bCancel_)
                    {
                      break;
                    }

                  MessageProcessingQueue queue{};

                  queue.swap(queue_);

                  lock.unlock();

                  for(auto & entry : queue)
                    {
                      try
                        {
                          // execute the funtor
                          entry();
                        }
                      catch(std::exception & exp)
                        {
                          // cannot really do too much at this point, so we'll log it
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
                          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                                  ERROR_LEVEL,
                                                  "%03hu NEMQueuedLayer::processWorkQueue:"
                                                  " Exception caught",
                                                  id_);
                        }
                    }

                  queue.clear();
                }
            }
          else
            {
              auto iter = fileDescriptorStore_.find(events[n].data.fd);

              if(iter != fileDescriptorStore_.end())
                {
                  try
                    {
                      iter->second.second(events[n].data.fd);
                    }
                  catch(std::exception & exp)
                    {
                      // cannot really do too much at this point, so we'll log it
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
                      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                              ERROR_LEVEL,
                                              "%03hu NEMQueuedLayer::processWorkQueue:"
                                              " Exception caught",
                                              id_);
                    }
                }
            }
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
                                                          DownstreamPacket & pkt,
                                                          const ControlMessages msgs)
{
  avgQueueWait_.update(std::chrono::duration_cast<Microseconds>(Clock::now() - enqueueTime).count());

  ++*pProcessedDownstreamPacket_;

  doProcessDownstreamPacket(pkt,msgs);

  std::for_each(msgs.begin(),msgs.end(),[](const ControlMessage * p){delete p;});
}

void EMANE::NEMQueuedLayer::handleProcessUpstreamPacket(TimePoint enqueueTime,
                                                        UpstreamPacket & pkt,
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


void EMANE::NEMQueuedLayer::updateTimerStats(TimePoint enqueueTime,
                                             const TimePoint & expireTime,
                                             const TimePoint & scheduleTime,
                                             const TimePoint & fireTime)
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
}

void EMANE::NEMQueuedLayer::handleProcessTimedEvent(TimePoint enqueueTime,
                                                    TimerEventId eventId,
                                                    const TimePoint & expireTime,
                                                    const TimePoint & scheduleTime,
                                                    const TimePoint & fireTime,
                                                    const void * arg)

{
  updateTimerStats(enqueueTime,expireTime,scheduleTime,fireTime);

  doProcessTimedEvent(eventId,expireTime,scheduleTime,fireTime,arg);
}


void  EMANE::NEMQueuedLayer::processTimer_i(TimerServiceProvider::TimerCallback callback,
                                            const TimePoint & expireTime,
                                            const TimePoint & scheduleTime,
                                            const TimePoint & fireTime)
{
  auto enqueueTime = Clock::now();

  enqueue_i([this,enqueueTime,callback,expireTime,scheduleTime,fireTime]()
            {
              updateTimerStats(enqueueTime,expireTime,scheduleTime,fireTime);
              callback(expireTime,scheduleTime,fireTime);
            });
}


void EMANE::NEMQueuedLayer::removeFileDescriptor(int iFd)
{
  auto iter = fileDescriptorStore_.find(iFd);

  if(iter != fileDescriptorStore_.end())
    {
      if(epoll_ctl(iepollFd_,EPOLL_CTL_DEL,iFd,nullptr) == -1)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "%03hu NEMQueuedLayer::NEMQueuedLayer:"
                                  " unable to add fd to epoll",
                                  id_);
        }

      fileDescriptorStore_.erase(iter);
    }
}

void EMANE::NEMQueuedLayer::addFileDescriptor_i(int iFd,
                                                DescriptorType type,
                                                Callback callback)
{
  auto iter = fileDescriptorStore_.find(iFd);

  if(iter == fileDescriptorStore_.end())
    {
      fileDescriptorStore_.insert(std::make_pair(iFd,std::make_pair(type,callback)));

      struct epoll_event ev;
      ev.events = type == DescriptorType::READ ? EPOLLIN : EPOLLOUT;
      ev.data.fd = iFd;

      if(epoll_ctl(iepollFd_,EPOLL_CTL_ADD,iFd,&ev) == -1)
        {
          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  ERROR_LEVEL,
                                  "%03hu NEMQueuedLayer::NEMQueuedLayer:"
                                  " unable to add fd to epoll",
                                  id_);
        }
    }
  else
    {
      if(iter->second.first != type)
        {
          iter->second.first = type;

          struct epoll_event ev;
          ev.events = type == DescriptorType::READ ? EPOLLIN : EPOLLOUT;
          ev.data.fd = iFd;

          if(epoll_ctl(iepollFd_,EPOLL_CTL_MOD,iFd,&ev) == -1)
            {
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "%03hu NEMQueuedLayer::NEMQueuedLayer:"
                                      " unable to add fd to epoll",
                                      id_);
            }
        }

      iter->second.second = callback;
    }
}
