/*
 * Copyright (c) 2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/utils/processingpool.h"

EMANE::Utils::ProcessingPool::ProcessingPool():
  bCancel_{}{}


EMANE::Utils::ProcessingPool::~ProcessingPool()
{
  stop();
}

void EMANE::Utils::ProcessingPool::start(std::size_t size)
{
  if(threads_.empty())
    {
      for(std::size_t i = 0; i < size; ++i)
        {
          threads_.push_back(std::thread{&ProcessingPool::worker,this});
        }
    }
}

void EMANE::Utils::ProcessingPool::stop()
{
  if(!bCancel_)
    {
      std::lock_guard<std::mutex> m{mutex_};

      bCancel_ = true;

      cond_.notify_all();
    }

  for(auto & thread : threads_)
    {
      thread.join();
    }

  threads_.clear();

  bCancel_ = false;
}

bool EMANE::Utils::ProcessingPool::isRunning() const
{
  return !threads_.empty();
}

void EMANE::Utils::ProcessingPool::worker()
{
  FunctionWrapper f;

  while(!bCancel_)
    {
      {
        std::unique_lock<std::mutex> m{mutex_};

        cond_.wait(m,[this]{return !queue_.empty() || bCancel_;});

        if(bCancel_)
          {
            break;
          }

        f = std::move(queue_.front());

        queue_.pop_front();
      }

      try
        {
          f();
        }
      catch(...)
        {

        }
    }
}
