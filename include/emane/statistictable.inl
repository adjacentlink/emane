/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/statistictableexception.h"

template<typename Key,typename Compare,std::size_t sortIndex>
EMANE::StatisticTable<Key,Compare,sortIndex>::StatisticTable(const StatisticTableLabels & labels):
  labels_{labels}{}

template<typename Key,typename Compare,std::size_t sortIndex>
EMANE::StatisticTable<Key,Compare,sortIndex>::~StatisticTable(){}


template<typename Key,typename Compare,std::size_t sortIndex>
void EMANE::StatisticTable<Key,Compare,sortIndex>::setCell(const Key & key,
                                         std::size_t columnIndex,
                                         const Any & any)
{
  std::lock_guard<std::mutex> m(mutex_);
  
  auto iter = table_.find(key);

  if(iter != table_.end())
    {
      if(columnIndex < labels_.size())
        {
          iter->second[columnIndex] = any;
        }
      else
        {
          throw makeException<StatisticTableException>("column index out of range: %zu",
                                                       columnIndex);
        }
    }
  else
    {
      throw StatisticTableException("unknown row key");
    }
}

template<typename Key,typename Compare,std::size_t sortIndex>
void EMANE::StatisticTable<Key,Compare,sortIndex>::setRow(const Key & key,
                                                          const std::vector<Any> & anys)
{
  std::lock_guard<std::mutex> m(mutex_);

  auto iter = table_.find(key);

  if(iter != table_.end())
    {
      if(anys.size() == labels_.size())
        {
          iter->second = anys;
        }
      else
        {
          throw makeException<StatisticTableException>("column count not valid: %zu",
                                                       anys.size());
        }
    }
  else
    {
      throw StatisticTableException("unknown row key");
    }
}

template<typename Key,typename Compare,std::size_t sortIndex>
void EMANE::StatisticTable<Key,Compare,sortIndex>::addRow(const Key & key,
                                                          const std::vector<Any> & anys)
{
  std::lock_guard<std::mutex> m(mutex_);

  if(anys.size() == labels_.size())
    {
      
      if(!table_.insert(std::make_pair(key,anys)).second)
        {
          throw StatisticTableException("duplicate row key");
        }
    }
  else if(anys.empty())
    {
      if(!table_.insert(std::make_pair(key,std::vector<Any>(labels_.size(),Any("")))).second)
        {
          throw StatisticTableException("duplicate row key");
        }
    }
  else
    {
      throw makeException<StatisticTableException>("column count not valid: %zu",
                                                   anys.size());

    }
}

template<typename Key,typename Compare,std::size_t sortIndex>
void EMANE::StatisticTable<Key,Compare,sortIndex>::deleteRow(const Key & key)
{
  std::lock_guard<std::mutex> m(mutex_);

  table_.erase(key);
}

template<typename Key,typename Compare,std::size_t sortIndex>
EMANE::StatisticTableValues
EMANE::StatisticTable<Key,Compare,sortIndex>::getValues() const
{
  StatisticTableValues values{};
  
  // lock guard block scope
  {
    std::lock_guard<std::mutex> m(mutex_);
    
    std::transform(table_.begin(),
                   table_.end(),
                   std::back_inserter(values),
                   std::bind(&InternalTable::value_type::second,
                             std::placeholders::_1));
  }
  Compare cmp{};
  std::sort(values.begin(),
            values.end(),
            [&cmp](const std::vector<Any> & a1,
                   const std::vector<Any> & a2)
            {
              return cmp(a1[sortIndex],a2[sortIndex]);
            });
  
  return values;
}

template<typename Key,typename Compare,std::size_t sortIndex>
EMANE::StatisticTableLabels
EMANE::StatisticTable<Key,Compare,sortIndex>::getLabels() const
{
  // labels_ is immutable - no synchronization required
  return labels_;
}

template<typename Key,typename Compare,std::size_t sortIndex>
void
EMANE::StatisticTable<Key,Compare,sortIndex>::clear()
{ 
  std::lock_guard<std::mutex> m(mutex_);
  table_.clear();
}
