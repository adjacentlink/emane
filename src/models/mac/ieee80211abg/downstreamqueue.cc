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


#include "downstreamqueue.h"
#include "macconfig.h"
#include "macstatistics.h"


/**
 *
 * @brief queue constructor
 * 
 */
EMANE::Models::IEEE80211ABG::DownstreamQueue::DownstreamQueue(EMANE::NEMId id):
  id_{id},
  numActiveCategories_{MAX_ACCESS_CATEGORIES},
  pNumUnicastPacketsUnsupported_{},
  pNumUnicastBytesUnsupported_{},
  pNumBroadcastPacketsUnsupported_{},
  pNumBroadcastBytesUnsupported_{}
{
  for(std::uint8_t u8Category = 0; u8Category < MAX_ACCESS_CATEGORIES; ++u8Category)
    {
      categories_[u8Category].category_ = u8Category;
    }
}


/**
 *
 * @brief queue destructor
 * 
 */
EMANE::Models::IEEE80211ABG::DownstreamQueue::~DownstreamQueue()
{ }



void EMANE::Models::IEEE80211ABG::DownstreamQueue::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  pNumUnicastPacketsUnsupported_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUnicastPacketsUnsupported",
                                                      StatisticProperties::CLEARABLE);
  pNumUnicastBytesUnsupported_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numUnicastBytesUnsupported",
                                                      StatisticProperties::CLEARABLE);
  pNumBroadcastPacketsUnsupported_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numBroadcastPacketsUnsupported",
                                                      StatisticProperties::CLEARABLE);
  pNumBroadcastBytesUnsupported_ =
    statisticRegistrar.registerNumeric<std::uint32_t>("numBroadcastBytesUnsupported",
                                                      StatisticProperties::CLEARABLE);

  for(std::uint8_t u8Category = 0; u8Category < MAX_ACCESS_CATEGORIES; ++u8Category)
    {
      categories_[u8Category].registerStatistics(statisticRegistrar);
    }
}



/**
 *
 * @brief set the max number of entries for a given queue index
 *
 * @param maxEntries max number of entrie size
 * @param u8Category queue index
 * 
 */
void
EMANE::Models::IEEE80211ABG::DownstreamQueue::setMaxCapacity(size_t maxEntries, std::uint8_t u8Category)
{
  if(u8Category < numActiveCategories_)
    {
      // clear entries that extend past the max limit
      while(maxEntries < categories_[u8Category].queue_.size())
        {
          categories_[u8Category].queue_.pop();
        }

      // set new size
      categories_[u8Category].u8MaxQueueCapacity_ = maxEntries;
    }
}



/**
 *
 * @brief set the max number of entries for all queues
 *
 * @param maxEntries max number of entrie size
 * 
 */
void
EMANE::Models::IEEE80211ABG::DownstreamQueue::setMaxCapacity(size_t maxEntries)
{
  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      // clear entries that extend past the max limit
      while(maxEntries < categories_[u8Category].queue_.size())
        {
          categories_[u8Category].queue_.pop();
        }

      // set new size
      categories_[u8Category].u8MaxQueueCapacity_ = maxEntries;
    }
}



/**
 *
 * @brief set the max entry size for a given queue index
 *
 * @param maxEntrySize max entry size
 * @param u8Category queue index
 * 
 */
void
EMANE::Models::IEEE80211ABG::DownstreamQueue::setMaxEntrySize(size_t maxEntrySize, std::uint8_t u8Category)
{
  // set new size
  categories_[u8Category].u16MaxPacketSize_ = maxEntrySize;
}



/**
 *
 * @brief set the max entry size for all queues
 *
 * @param maxEntrySize max entry size
 * 
 */
void
EMANE::Models::IEEE80211ABG::DownstreamQueue::setMaxEntrySize(size_t maxEntrySize)
{
  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      // set new size
      categories_[u8Category].u16MaxPacketSize_ = maxEntrySize;
    }
}


/**
 *
 * @brief get the max number of entries for a given queue index
 *
 * @param u8Category queue index
 *
 * @retval max number of entries
 * 
 */
size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getMaxCapacity(std::uint8_t u8Category)
{
  size_t result{};

  if(u8Category < numActiveCategories_)
    {
      result = categories_[u8Category].u8MaxQueueCapacity_;
    }

  return result;
}

/**
 *
 * @brief get the max number of entries for all queues
 *
 * @retval max number of entries
 * 
 */
size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getMaxCapacity()
{
  size_t result{};

  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      result += categories_[u8Category].u8MaxQueueCapacity_;
    }

  return result;
}



/**
 *
 * @brief get the number of entries for a given queue index
 *
 * @param u8Category queue index
 *
 * @retval number of entries
 * 
 */
size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getDepth(std::uint8_t u8Category)
{
  size_t result{};
  
  if(u8Category < numActiveCategories_)
    {
      result = categories_[u8Category].queue_.size();
    }

  return result;
}

/**
 *
 * @brief get the number of entries for all active queues
 *
 * @retval number of entries
 * 
 */
size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getDepth()
{
  size_t result{};

  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      result += categories_[u8Category].queue_.size();
    }

  return result;
}



size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getAvailableSpace(std::uint8_t u8Category)
{
  size_t result{};

  if(u8Category < numActiveCategories_)
    {
      result = categories_[u8Category].u8MaxQueueCapacity_ - categories_[u8Category].queue_.size();
    }

  return result;
}


size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getAvailableSpace()
{
  size_t result{};

  // total queue size
  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      // space remaining
      result += categories_[u8Category].u8MaxQueueCapacity_ - categories_[u8Category].queue_.size();
    }

  return result;
}



/**
 *
 * @brief set the number of categories (queues)
 *
 * @param u8NumCategories
 *
 */
void
EMANE::Models::IEEE80211ABG::DownstreamQueue::setCategories(std::uint8_t u8NumCategories)
{
  // check min/max range 1 or more
  if((u8NumCategories > 0) && (u8NumCategories <= MAX_ACCESS_CATEGORIES))
    {
      for(std::uint8_t u8Category = u8NumCategories; u8Category < numActiveCategories_; ++u8Category)
        {
          // clear queues that will no longer be used
          while(categories_[numActiveCategories_ - u8Category].queue_.empty() == false)
            {
              categories_[numActiveCategories_ - u8Category].queue_.pop();
            }
        }

      // set new size
      numActiveCategories_ = u8NumCategories;
    }
}


/**
 *
 * @brief blocking dequeue, returns highest priority item first
 *
 * @retval queue entry
 *
 */
std::pair<EMANE::Models::IEEE80211ABG::DownstreamQueueEntry, bool>
EMANE::Models::IEEE80211ABG::DownstreamQueue::dequeue()
{
  // try higher priority first, work down to lower priority
  for(int iIndex = numActiveCategories_ - 1; iIndex >= 0; --iIndex)
    {
      // queue not empty
      if(categories_[iIndex].queue_.empty() == false)
        {
          DownstreamQueueEntry entry{std::move(categories_[iIndex].queue_.front())};

          // pop entry 
          categories_[iIndex].queue_.pop();
          
          return {std::move(entry),true};
        }
    }

  return {DownstreamQueueEntry(),false};
}

/**
 *
 * @brief enqueue, inserts items by priority, signals on success.
 *
 */
std::vector<EMANE::Models::IEEE80211ABG::DownstreamQueueEntry> 
EMANE::Models::IEEE80211ABG::DownstreamQueue::enqueue(DownstreamQueueEntry & entry)
{
  std::vector<DownstreamQueueEntry> result;

  if(entry.u8Category_ < numActiveCategories_)
    {
      AccessCategory * p{&categories_[entry.u8Category_]};

      // queue disabled if max size for this category is 0
      if(p->u8MaxQueueCapacity_ == 0)
        {
          // add to drop list 
          result.push_back(std::move(entry));
        }
      // if max msdu is enabled and entry too large
      else if((p->u16MaxPacketSize_ != 0) && (entry.pkt_.length() > p->u16MaxPacketSize_))
        {
          // bump entry exceeded msdu
          if(entry.pkt_.getPacketInfo().getDestination() == EMANE::NEM_BROADCAST_MAC_ADDRESS)
            {
              ++*p->pNumBroadcastPacketsTooLarge_;
              p->pNumBroadcastBytesTooLarge_ += entry.pkt_.length();
            }
          else
            {
              ++*p->pNumUnicastPacketsTooLarge_;
              *p->pNumUnicastBytesTooLarge_ += entry.pkt_.length();
            }

          // add to drop list
          result.push_back(std::move(entry));
        }
      else
        {
          // check for queue overflow
          while(p->queue_.size() >= p->u8MaxQueueCapacity_)
            {
              // add to drop list
              result.push_back(std::move(p->queue_.front()));

              // pop entry
              p->queue_.pop();
            }

          p->queue_.push(std::move(entry));

          // bump high water mark
          if(p->queue_.size() > p->pNumHighWaterMark_->get())
            {
              *p->pNumHighWaterMark_ = p->queue_.size();
              *p->pNumHighWaterMax_ = p->u8MaxQueueCapacity_;
            }
        }
    }
  else
    {
      if(entry.pkt_.getPacketInfo().getDestination() == EMANE::NEM_BROADCAST_MAC_ADDRESS)
        {
          ++*pNumBroadcastPacketsUnsupported_;
          *pNumBroadcastBytesUnsupported_ += entry.pkt_.length();
        }
      else
        {
          ++*pNumUnicastPacketsUnsupported_;
          *pNumUnicastBytesUnsupported_ += entry.pkt_.length();
        }

      // add to drop list 
      result.push_back(std::move(entry));
    }

  return result;
}



/* @param u8Category queue index
 * @param bClear clear num discard history
 *
 * @retval max number of entries
 * 
 */
size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getNumOverFlow(std::uint8_t u8Category, bool bClear)
{
  size_t result{};

  if(u8Category < numActiveCategories_)
    {
      result = categories_[u8Category].numPacketOverFlow_;

      if(bClear == true)
        {
          categories_[u8Category].numPacketOverFlow_ = 0;
        }
    }

  return result;
}


size_t 
EMANE::Models::IEEE80211ABG::DownstreamQueue::getNumOverFlow(bool bClear)
{
  size_t result{};

  for(std::uint8_t u8Category = 0; u8Category < numActiveCategories_; ++u8Category)
    {
      result += categories_[u8Category].numPacketOverFlow_;

      if(bClear == true)
        {
          categories_[u8Category].numPacketOverFlow_ = 0;
        }
    }

  return result;
}
