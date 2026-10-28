/*
 * Copyright (c) 2025-2026 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "remoteactivityservicebackend.h"
#include "remotetimerbackend.h"
#include "timerservice.h"
#include "logservice.h"

#include "simactivityservice/topics.h"
#include "simactivityservice.pb.h"

#include "emane/startexception.h"

#include <vector>
#include <sstream>

using namespace SimActivityService::Topics;

EMANE::RemoteActivityServiceBackend::
RemoteActivityServiceBackend(const uuid_t & uuid,
                             const INETAddr & simulationCoonrdinatorEndpoint,
                             const INETAddr & activityServiceEndpoint,
                             const std::string & sActivityServiceTag):
  sTag_{sActivityServiceTag},
  mutex_{},
  mutexSync_{},
  iEventFd_{},
  simulationTime_{},
  bStarted_{},
  u16WorkQueueDepth_{},
  u64SequenceNumber_{},
  u64ActionsSinceLastStatus_{},
  lastStatusTime_{TimePoint::max()},
  bNotFirstSignal_{false}
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  uuid_copy(uuid_,uuid);

  iEventFd_ = eventfd(0,0);

  pContext_.reset(zmq_ctx_new());

  if(!pContext_)
    {
      throw makeException<StartException>("Error creating messaging context: %s",
                                          zmq_strerror(errno));
    }

  int iIPv6 = 1;

  pSubSocket_.reset(zmq_socket(pContext_.get(),ZMQ_SUB));

  if(!pSubSocket_)
    {
      throw makeException<StartException>("Error creating new sub socket: %s",
                                          zmq_strerror(errno));
    }

  if(zmq_setsockopt(pSubSocket_.get(),ZMQ_IPV6,&iIPv6,sizeof(iIPv6)))
    {
      throw makeException<StartException>("Error setting sub socket opiton ZMQ_IPV6: %s",
                                          zmq_strerror(errno));
    }

  if(zmq_connect(pSubSocket_.get(),
                 std::string{"tcp://"}.append(simulationCoonrdinatorEndpoint.str(true)).c_str()))
    {
      throw makeException<StartException>("Error connecting sub socket: %s",
                                          zmq_strerror(errno));
    }


  std::vector<std::string> topics{ACTIVITY_SERVICE_TOPIC_HELLO_ANNOUNCE,
                                  ACTIVITY_SERVICE_TOPIC_TIME_NOTICE,
                                  ACTIVITY_SERVICE_TOPIC_TIME_NOW,
                                  ACTIVITY_SERVICE_TOPIC_TIME_SIGNAL};

  for(const auto & topic : topics)
    {
      if(zmq_setsockopt(pSubSocket_.get(),
                        ZMQ_SUBSCRIBE,
                        topic.c_str(),
                        topic.size()))
        {
          throw makeException<StartException>("Error subscribing %s: %s",
                                              topic,
                                              zmq_strerror(errno));
        }
    }

  pPubSocket_.reset(zmq_socket(pContext_.get(),ZMQ_PUB));

  if(!pPubSocket_)
    {
      throw makeException<StartException>("Error creating new pub socket: %s",
                                          zmq_strerror(errno));
    }

  iIPv6 = 1;

  if(zmq_setsockopt(pPubSocket_.get(),ZMQ_IPV6,&iIPv6,sizeof(iIPv6)))
    {
      throw makeException<StartException>("Error setting socket opiton ZMQ_IPV6: %s",
                                          zmq_strerror(errno));
    }

  if(zmq_bind(pPubSocket_.get(),
              std::string{"tcp://"}.append(activityServiceEndpoint.str(true)).c_str()))
    {
      throw makeException<StartException>("Error binding pub socket: %s",
                                          zmq_strerror(errno));
    }

  mutexSync_.lock();

  pTimerBackend_.reset(new RemoteTimerBackend{});

  TimerServiceSingleton::instance()->setBackend(pTimerBackend_.get());

  thread_ = std::move(std::thread(&RemoteActivityServiceBackend::process,this));
}

EMANE::RemoteActivityServiceBackend::~RemoteActivityServiceBackend()
{
  if(thread_.joinable())
    {
      write(iEventFd_,&one,sizeof(one));

      thread_.join();
    }
}

EMANE::Clock::time_point EMANE::RemoteActivityServiceBackend::now()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);
  return simulationTime_;
}

void
EMANE::RemoteActivityServiceBackend::process()
{
  LogServiceSingleton::instance()->log(INFO_LEVEL,
                                       "RemoteActivityServiceBackend::%s",
                                       __func__);

  bool bRun{true};

  while(bRun)
    {
      try
        {
          std::vector<zmq_pollitem_t> items =
            {
              {nullptr,iEventFd_,ZMQ_POLLIN,0},
              {pSubSocket_.get(),0,ZMQ_POLLIN,0},
            };

          int rc = zmq_poll(&items[0],items.size(),-1);

          if(rc == -1)
            {
              continue;
            }

          for(const auto & item : items)
            {
              // process internal messages between frontend and backend
              if(item.revents & ZMQ_POLLIN)
                {
                  if(item.fd == iEventFd_)
                    {
                      bRun = false;

                      break;
                    }
                  else if(item.socket == pSubSocket_.get())
                    {
                      std::lock_guard<std::recursive_mutex> guard(mutex_);

                      std::string sMessageTopic{};

                      while(1)
                        {
                          zmq_msg_t message;

                          zmq_msg_init(&message);

                          zmq_msg_recv(&message,item.socket, 0);

                          int iMore{};

                          size_t sizeMore{sizeof(iMore)};

                          zmq_getsockopt(item.socket,ZMQ_RCVMORE,&iMore,&sizeMore);

                          if(iMore)
                            {
                              sMessageTopic =
                                std::string{reinterpret_cast<char *>(zmq_msg_data(&message)),zmq_msg_size(&message)};

                              LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                                                   "RemoteActivityServiceBackend::%s"
                                                                   " sub %s",
                                                                   __func__,
                                                                   sMessageTopic.c_str());
                            }
                          else
                            {
                              SimActivityService::Activity m{};

                              if(m.ParseFromArray(zmq_msg_data(&message),
                                                  zmq_msg_size(&message)))
                                {
                                  if(m.type() == SimActivityService::Activity::TYPE_ACTIVITY_HELLO)
                                    {
                                      LogServiceSingleton::instance()->
                                        log(DEBUG_LEVEL,
                                            "RemoteActivityServiceBackend::%s"
                                            " topic: %s"
                                            " tag: %s",
                                            __func__,
                                            sMessageTopic.c_str(),
                                            m.tag().c_str());

                                      sendHello();
                                    }
                                  else if(m.type() == SimActivityService::Activity::TYPE_ACTIVITY_TIME_NOTICE)
                                    {
                                      if(m.has_time_notice())
                                        {
                                          const auto pTimeNotice = m.mutable_time_notice();

                                          LogServiceSingleton::instance()->
                                            log(DEBUG_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " topic: %s"
                                                " simulation message time: %zu"
                                                " simulation now time: %zu"
                                                " depth: %hu" ,
                                                __func__,
                                                sMessageTopic.c_str(),
                                                pTimeNotice->time_microseconds_since_epoch(),
                                                std::chrono::duration_cast<std::chrono::microseconds>
                                                (simulationTime_.time_since_epoch()).count(),
                                                u16WorkQueueDepth_);

                                          SimActivityService::Activity msg{};

                                          msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_TIME_CONFIRM_NOTICE);

                                          auto pTimeConfirmNotice = msg.mutable_time_confirm_notice();

                                          pTimeConfirmNotice->
                                            set_time_microseconds_since_epoch(pTimeNotice->time_microseconds_since_epoch());

                                          pTimeConfirmNotice->set_ready(!u16WorkQueueDepth_);

                                          if(u16WorkQueueDepth_)
                                            {
                                              notifyNOP();
                                            }

                                          pTimeConfirmNotice->set_reference_sequence_number(m.sequence_number());

                                          send(ACTIVITY_SERVICE_TOPIC_TIME_CONFIRM_NOTICE,msg);
                                        }
                                      else
                                        {
                                          LogServiceSingleton::instance()->
                                            log(ERROR_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " simulation message missing time_now"
                                                " field topic: %s, ignoring",
                                                __func__,
                                                sMessageTopic.c_str());
                                        }
                                    }
                                  else if(m.type() == SimActivityService::Activity::TYPE_ACTIVITY_TIME_NOW)
                                    {
                                      if(m.has_time_now())
                                        {
                                          const auto pTimeNow = m.mutable_time_now();

                                          // update the current time
                                          // provided it is not in the
                                          // past
                                          auto timeFromMessage =
                                            Clock::time_point{Microseconds{pTimeNow->time_microseconds_since_epoch()}};

                                          LogServiceSingleton::instance()->
                                            log(DEBUG_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " topic: %s"
                                                " simulation message time: %zu"
                                                " simulation now time: %zu"
                                                " depth: %hu"
                                                " actions since last status: %zu" ,
                                                __func__,
                                                sMessageTopic.c_str(),
                                                pTimeNow->time_microseconds_since_epoch(),
                                                std::chrono::duration_cast<std::chrono::microseconds>
                                                (simulationTime_.time_since_epoch()).count(),
                                                u16WorkQueueDepth_,
                                                u64ActionsSinceLastStatus_);


                                          if(!bStarted_ || timeFromMessage > simulationTime_)
                                            {
                                              LogServiceSingleton::instance()->
                                                log(DEBUG_LEVEL,
                                                    "RemoteActivityServiceBackend::%s"
                                                    " simulation message time %zu depth %hu" ,
                                                    __func__,
                                                    pTimeNow->time_microseconds_since_epoch(),
                                                    u16WorkQueueDepth_);

                                              simulationTime_ = std::move(timeFromMessage);

                                              SimActivityService::Activity msg{};

                                              msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_TIME_CONFIRM_NOW);

                                              auto pTimeConfirmNow = msg.mutable_time_confirm_now();

                                              pTimeConfirmNow->
                                                set_time_microseconds_since_epoch(std::chrono::duration_cast
                                                                                  <std::chrono::microseconds>(simulationTime_.time_since_epoch()).count());

                                              pTimeConfirmNow->set_ready(true);

                                              pTimeConfirmNow->set_reference_sequence_number(m.sequence_number());

                                              send(ACTIVITY_SERVICE_TOPIC_TIME_CONFIRM_NOW,msg);

                                              if(!bStarted_)
                                                {
                                                  mutexSync_.unlock();

                                                  bStarted_ = true;
                                                }
                                            }
                                          else
                                            {
                                              LogServiceSingleton::instance()->
                                                log(ERROR_LEVEL,
                                                    "RemoteActivityServiceBackend::%s"
                                                    " simulation message time"
                                                    " not in the future now: %zu"
                                                    " new: %zu, ignorng",
                                                    __func__,
                                                    std::chrono::duration_cast<std::chrono::microseconds>
                                                    (simulationTime_.time_since_epoch()).count(),
                                                    pTimeNow->time_microseconds_since_epoch());
                                            }
                                        }
                                      else
                                        {
                                          LogServiceSingleton::instance()->
                                            log(ERROR_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " simulation message missing time_now"
                                                " field topic: %s, ignoring",
                                                __func__,
                                                sMessageTopic.c_str());
                                        }
                                    }
                                  else if(m.type() == SimActivityService::Activity::TYPE_ACTIVITY_TIME_SIGNAL)
                                    {
                                      bool bSendStatus{};

                                      auto earliestExpireTime =
                                        pTimerBackend_->getEarliestExpireTime();

                                      if(!earliestExpireTime.has_value() ||
                                         earliestExpireTime.value() > simulationTime_ ||
                                         simulationTime_ == TimePoint{Microseconds{0}} )
                                        {
                                          bSendStatus = true;
                                        }

                                      LogServiceSingleton::instance()->
                                        log(DEBUG_LEVEL,
                                            "RemoteActivityServiceBackend::%s"
                                            " timer has value: %s"
                                            " timer earliest: %zu"
                                            " simulation time: %zu",
                                            __func__,
                                            earliestExpireTime.has_value() ? "yes" : "no",
                                            earliestExpireTime.has_value() ? std::chrono::duration_cast<std::chrono::microseconds>
                                            ( earliestExpireTime.value().time_since_epoch()).count() : 0,
                                            std::chrono::duration_cast<std::chrono::microseconds>
                                            (simulationTime_.time_since_epoch()).count());

                                      pTimerBackend_->signal();

                                      if(bSendStatus && lastStatusTime_ != simulationTime_)
                                        {
                                          LogServiceSingleton::instance()->
                                            log(DEBUG_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " sending status next time 0,"
                                                " still no work to do",
                                                __func__);

                                          SimActivityService::Activity msg{};

                                          msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_STATUS);

                                          auto pStatus = msg.mutable_status();

                                          if(earliestExpireTime.has_value())
                                            {
                                              pStatus->
                                                set_next_time_microseconds_since_epoch(std::chrono::duration_cast<std::chrono::microseconds>
                                                                                       (earliestExpireTime.value().time_since_epoch()).count());
                                            }
                                          else
                                            {
                                              pStatus->set_next_time_microseconds_since_epoch(0);
                                            }

                                          LogServiceSingleton::instance()->
                                            log(DEBUG_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " sending: %s next: %zu",
                                                __func__,
                                                ACTIVITY_SERVICE_TOPIC_STATUS,
                                                pStatus->next_time_microseconds_since_epoch());

                                          lastStatusTime_ = simulationTime_;

                                          if(bNotFirstSignal_)
                                            {
                                              u64ActionsSinceLastStatus_ = 0;

                                              send(ACTIVITY_SERVICE_TOPIC_STATUS,msg);
                                            }
                                          else
                                            {
                                              bNotFirstSignal_ = true;

                                              if(!u64ActionsSinceLastStatus_)
                                                {
                                                  send(ACTIVITY_SERVICE_TOPIC_STATUS,msg);
                                                }
                                            }
                                        }
                                      else
                                        {
                                          LogServiceSingleton::instance()->
                                            log(DEBUG_LEVEL,
                                                "RemoteActivityServiceBackend::%s"
                                                " simulation message time"
                                                " not sending status"
                                                " depth: %hu"
                                                " earliest has value: %s"
                                                " earliest: %zu"
                                                " simulation time: %zu"
                                                " last status time: %zu",
                                                __func__,
                                                u16WorkQueueDepth_,
                                                earliestExpireTime.has_value() ? "yes" : "no",
                                                earliestExpireTime.has_value() ? std::chrono::duration_cast<std::chrono::microseconds>
                                                (earliestExpireTime.value().time_since_epoch()).count() : 0,
                                                std::chrono::duration_cast<std::chrono::microseconds>
                                                (simulationTime_.time_since_epoch()).count(),
                                                std::chrono::duration_cast<std::chrono::microseconds>
                                                (lastStatusTime_.time_since_epoch()).count());

                                          if(lastStatusTime_ != simulationTime_)
                                            {
                                              notifyNOP();
                                            }
                                        }
                                    }
                                }
                              else
                                {
                                  LogServiceSingleton::instance()
                                    ->log(ERROR_LEVEL,
                                          "RemoteActivityServiceBackend::%s"
                                          " unable to pasre simulation message"
                                          " topic: %s, ignoring",
                                          __func__,
                                          sMessageTopic.c_str());
                                }
                            }

                          zmq_msg_close(&message);

                          if(!iMore)
                            {
                              break;
                            }
                        }
                    }
                }
            }
        }
      catch(std::exception & exp)
        {

          LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                               "RemoteActivityServiceBackend::%s %s",
                                               __func__,
                                               exp.what());
        }
    }
}

void
EMANE::RemoteActivityServiceBackend::waitForStart()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s begin",
                                       __func__);

  mutexSync_.lock();

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s end",
                                       __func__);
}


void
EMANE::RemoteActivityServiceBackend::cancelWaitForStart()
{
  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s begin",
                                       __func__);

  mutexSync_.unlock();

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s 1",
                                       __func__);
}

bool
EMANE::RemoteActivityServiceBackend::isStarted()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  return bStarted_;
}

void EMANE::RemoteActivityServiceBackend::notifyWorkEnqueue()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s depth:"
                                       " %hu (pre increment)",
                                       __func__,
                                       u16WorkQueueDepth_);


  if(u16WorkQueueDepth_ == std::numeric_limits<std::uint16_t>::max())
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                           "RemoteActivityServiceBackend::%s"
                                           " work queue depth is at max %hu cannot"
                                           " enqueue, ignoring",
                                           __func__,
                                           u16WorkQueueDepth_);
    }
  else
    {
      ++u16WorkQueueDepth_;
    }
}

void EMANE::RemoteActivityServiceBackend::notifyWorkDequeue()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s depth:"
                                       " %hu (pre decrement)",
                                       __func__,
                                       u16WorkQueueDepth_);


  if(u16WorkQueueDepth_ == 0)
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                           "RemoteActivityServiceBackend::%s"
                                           " work queue depth is %hu cannot"
                                           " dequeue, ignoring",
                                           __func__,
                                           u16WorkQueueDepth_);
    }
  else
    {
      --u16WorkQueueDepth_;

      // transitioned to an empty work queue
      LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                           "RemoteActivityServiceBackend::%s"
                                           " work queue depth: %hu"
                                           " actions since last status: %zu",
                                           __func__,
                                           u16WorkQueueDepth_,
                                           u64ActionsSinceLastStatus_);

      if(!u16WorkQueueDepth_ && u64ActionsSinceLastStatus_)
        {
          // send status message
          auto earliestExpireTime = pTimerBackend_->getEarliestExpireTime();

          SimActivityService::Activity msg{};

          msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_STATUS);

          auto pStatus = msg.mutable_status();

          if(earliestExpireTime && *earliestExpireTime != simulationTime_)
            {
              pStatus->
                set_next_time_microseconds_since_epoch(std::chrono::duration_cast<std::chrono::microseconds>
                                                       (earliestExpireTime->time_since_epoch()).count());
            }
          else
            {
              pStatus->set_next_time_microseconds_since_epoch(0);
            }


          LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                               "RemoteActivityServiceBackend::%s"
                                               " sending status: %s next: %zu",
                                               __func__,
                                               ACTIVITY_SERVICE_TOPIC_STATUS,
                                               pStatus->next_time_microseconds_since_epoch());

          lastStatusTime_ = simulationTime_;

          send(ACTIVITY_SERVICE_TOPIC_STATUS,msg);

          u64ActionsSinceLastStatus_ = 0;
        }
    }
}

void EMANE::RemoteActivityServiceBackend::notifyOTA(const uuid_t & uuid,
                                                    std::uint64_t u64SequenceNumber,
                                                    NEMId source,
                                                    NEMId destination)
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  char pzUUID[37];

  uuid_unparse(uuid,pzUUID);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s"
                                       " uuid: %s"
                                       " sequence: %zu"
                                       " source: %hu"
                                       " destination: %hu"
                                       " now: %zu",
                                       __func__,
                                       pzUUID,
                                       u64SequenceNumber,
                                       source,
                                       destination,
                                       std::chrono::duration_cast<std::chrono::microseconds>
                                       (simulationTime_.time_since_epoch()).count());

  SimActivityService::Activity msg{};

  msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_ACTION);

  auto pAction = msg.mutable_action();

  pAction->set_name("emane.ota");

  std::stringstream ssIdentifer{};

  ssIdentifer<<pzUUID
             <<"-"
             <<u64SequenceNumber
             <<"-"
             <<source
             <<"-"
             <<destination;

  pAction->set_unique_identifier(ssIdentifer.str());

  send(ACTIVITY_SERVICE_TOPIC_ACTION,msg);

  ++u64ActionsSinceLastStatus_;
}

void
EMANE::RemoteActivityServiceBackend::notifyEvent(const uuid_t & uuid,
                                                 std::uint64_t u64SequenceNumber)
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  char pzUUID[37];

  uuid_unparse(uuid,pzUUID);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s"
                                       " uuid: %s"
                                       " sequence: %zu"
                                       " now: %zu",
                                       __func__,
                                       pzUUID,
                                       u64SequenceNumber,
                                       std::chrono::duration_cast<std::chrono::microseconds>
                                       (simulationTime_.time_since_epoch()).count());

  SimActivityService::Activity msg{};

  msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_ACTION);

  auto pAction = msg.mutable_action();

  pAction->set_name("emane.event");

  std::stringstream ssIdentifer{};

  ssIdentifer<<pzUUID
             <<"-"
             <<u64SequenceNumber;

  pAction->set_unique_identifier(ssIdentifer.str());

  send(ACTIVITY_SERVICE_TOPIC_ACTION,msg);

  ++u64ActionsSinceLastStatus_;
}

void
EMANE::RemoteActivityServiceBackend::notifyBoundary(NEMId source,
                                                    NEMId destination,
                                                    NEMId local,
                                                    std::uint64_t u64SequenceNumber)
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s"
                                       " source: %hu"
                                       " destination: %hu"
                                       " local: %hu"
                                       " sequence: %zu"
                                       " now: %zu",
                                       __func__,
                                       source,
                                       destination,
                                       local,
                                       u64SequenceNumber,
                                       std::chrono::duration_cast<std::chrono::microseconds>
                                       (simulationTime_.time_since_epoch()).count());


  SimActivityService::Activity msg{};

  msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_ACTION);

  auto pAction = msg.mutable_action();

  pAction->set_name("emane.boundary." + std::to_string(local));

  std::stringstream ssIdentifer{};

  ssIdentifer<<source
             <<"-"
             <<destination
             <<"-"
             <<local
             <<"-"
             <<u64SequenceNumber;

  pAction->set_unique_identifier(ssIdentifer.str());

  send(ACTIVITY_SERVICE_TOPIC_ACTION,msg);

  ++u64ActionsSinceLastStatus_;
}

void EMANE::RemoteActivityServiceBackend::notifyNOP()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s",
                                       __func__);
  ++u64ActionsSinceLastStatus_;
}

void
EMANE::RemoteActivityServiceBackend::notifyAction(const std::string & sName,
                                                  const std::string & sIdentifier)


{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s"
                                       " name: %s"
                                       " identifier: %s"
                                       " now: %zu",
                                       __func__,
                                       sName.c_str(),
                                       sIdentifier.c_str(),
                                       std::chrono::duration_cast<std::chrono::microseconds>
                                       (simulationTime_.time_since_epoch()).count());


  SimActivityService::Activity msg{};

  msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_ACTION);

  auto pAction = msg.mutable_action();

  pAction->set_name(sName);

  pAction->set_unique_identifier(sIdentifier);

  send(ACTIVITY_SERVICE_TOPIC_ACTION,msg);

  ++u64ActionsSinceLastStatus_;
}

void
EMANE::RemoteActivityServiceBackend::sendHello()
{
  std::lock_guard<std::recursive_mutex> m(mutex_);

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s",
                                       __func__);

  SimActivityService::Activity msg{};

  msg.set_type(SimActivityService::Activity::TYPE_ACTIVITY_HELLO);

  send(ACTIVITY_SERVICE_TOPIC_HELLO_ACKNOWLEDGE,msg);
}

void
EMANE::RemoteActivityServiceBackend::send(const std::string & sTopic,
                                          SimActivityService::Activity & msg)
{
  std::uint64_t u64TimeNowMicroseconds =
    std::chrono::duration_cast<std::chrono::microseconds>
    (simulationTime_.time_since_epoch()).count();

  LogServiceSingleton::instance()->log(DEBUG_LEVEL,
                                       "RemoteActivityServiceBackend::%s"
                                       " topic: %s"
                                       " now: %zu"
                                       " sequence: %zu",
                                       __func__,
                                       sTopic.c_str(),
                                       u64TimeNowMicroseconds,
                                       u64SequenceNumber_);

  msg.set_uuid(reinterpret_cast<const char *>(uuid_),sizeof(uuid_));

  msg.set_tag(sTag_);

  msg.set_time_microseconds_since_epoch(u64TimeNowMicroseconds);

  msg.set_sequence_number(u64SequenceNumber_);

  ++u64SequenceNumber_;

  std::string sSerialization{};

  if(msg.SerializeToString(&sSerialization))
    {
      if(zmq_send(pPubSocket_.get(),
                  sTopic.c_str(),
                  sTopic.length(),
                  ZMQ_SNDMORE) < 0 ||
         zmq_send(pPubSocket_.get(),
                  sSerialization.c_str(),
                  sSerialization.length(),
                  0) < 0)
        {
          LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                               "RemoteActivityServiceBackend"
                                               "::%s zmq send error %s",
                                               __func__,
                                               zmq_strerror(errno));
        }
    }
  else
    {
      LogServiceSingleton::instance()->log(ERROR_LEVEL,
                                           "RemoteActivityServiceBackend"
                                           "::%s unable to serialize"
                                           " topic %s",
                                           __func__,
                                           sTopic.c_str());


    }
}
