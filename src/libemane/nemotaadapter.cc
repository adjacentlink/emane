/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater,
 * New Jersey
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#include "nemotaadapter.h"
#include "otamanager.h"
#include "logservice.h"

#include "emane/utils/threadutils.h"
#include "emane/upstreamtransport.h"

EMANE::NEMOTAAdapter::NEMOTAAdapter(NEMId id):
  id_{id},
  bCancel_{}
{}

void EMANE::NEMOTAAdapter::open()
{
  bCancel_ = false;

  OTAManagerSingleton::instance()->registerOTAUser(id_,this);

  thread_ = std::thread{&NEMOTAAdapter::processPacketQueue,this};

  if(ThreadUtils::elevate(thread_))
    {
      LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                              ERROR_LEVEL,"NEMOTAAdapter::NEMOTAAdapter: Unable to set Priority");
    }
}

EMANE::NEMOTAAdapter::~NEMOTAAdapter()
{
  if(!bCancel_ && thread_.joinable())
    {
      mutex_.lock();
      bCancel_ = true;
      cond_.notify_one();
      mutex_.unlock();
      thread_.join();
    }
}

void EMANE::NEMOTAAdapter::close()
{
  try
    {
      OTAManagerSingleton::instance()->unregisterOTAUser(id_);
    }
  catch(...)
    {
    }

  if(thread_.joinable())
    {
      mutex_.lock();
      bCancel_ = true;
      cond_.notify_one();
      mutex_.unlock();
      thread_.join();
    }
}

void EMANE::NEMOTAAdapter::processOTAPacket(UpstreamPacket & pkt, const ControlMessages & msgs)
{
  sendUpstreamPacket(pkt, msgs);
}

void EMANE::NEMOTAAdapter::processDownstreamPacket(DownstreamPacket & pkt,
                                                   const ControlMessages & msgs)
{
  std::lock_guard<std::mutex> m(mutex_);

  queue_.emplace_back(pkt, msgs);

  cond_.notify_one();

}

void EMANE::NEMOTAAdapter::processPacketQueue()
{
  while(1)
    {
      std::unique_lock<std::mutex> lock(mutex_);

      while(queue_.empty() && !bCancel_)
        {
          cond_.wait(lock);
        }

      if(bCancel_)
        {
          break;
        }

      DownstreamPacketQueue queue{};

      queue.swap(queue_);

      lock.unlock();

      for(auto & entry : queue)
        {
          try
            { // id, pkt, ctrl
              OTAManagerSingleton::instance()->sendOTAPacket(id_, entry.first, entry.second);
            }
          catch(std::exception & exp)
            {
              // cannot really do too much at this point, so we'll log it
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEMOTAAdapter::processPacketQueue Excepetion caught: %s",
                                      exp.what());
            }
          catch(...)
            {
              // cannot really do too much at this point, so we'll log it
              LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                      ERROR_LEVEL,
                                      "NEMOTAAdapter::processPacketQueue Excepetion caught");
            }
        }

      queue.clear();
    }
}
