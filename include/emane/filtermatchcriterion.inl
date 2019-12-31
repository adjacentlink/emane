/*
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <emane/filtermatchcriterion.h>

namespace EMANE
{
  template<class T,
           class Compare,
           FilterElement element>
  FilterMatchCriterionCompare<T,Compare,element> *
  FilterMatchCriterionCompare<T,Compare,element>::create(T value)
  {
    return new FilterMatchCriterionCompare<T,Compare,element>{value};
  }

  template<class T,
           class Compare,
           FilterElement element>
  bool FilterMatchCriterionCompare<T,Compare,element>::operator()(const FilterElementValues & values) const
  {
    Compare cmp{};

    switch(element_)
      {
      case FilterElement::FREQUENCY:
        return cmp(values.u64FrequencyHz_,value_);
      case FilterElement::BANDWIDTH:
        return cmp(values.u64BandwidthHz_,value_);
      case FilterElement::SUBID:
        return cmp(values.u16SubId_,value_);
      default:
        break;
      }
    return false;
  }

  template<class T,
           class Compare,
           FilterElement element>
  FilterMatchCriterionCompare<T,Compare,element> *
  FilterMatchCriterionCompare<T,Compare,element>::clone() const
  {
    return new FilterMatchCriterionCompare<T,Compare,element>{value_};
  }

  template<class Compare,
           FilterElement element>
  FilterMatchCriterionCompare<std::string,Compare,element> *
  FilterMatchCriterionCompare<std::string,Compare,element>::create(const std::string & value)
  {
    return new FilterMatchCriterionCompare<std::string,Compare,element>{value};
  }

  template<class Compare,
           FilterElement element>
  bool FilterMatchCriterionCompare<std::string,Compare,element>::operator()(const FilterElementValues & values) const
  {
    Compare cmp{};

    switch(element_)
      {
      case FilterElement::FILTER_DATA:
        return cmp(values.filterData_,value_);
      default:
        break;
      }
    return false;
  }

  template<class Compare,
           FilterElement element>
  FilterMatchCriterionCompare<std::string,Compare,element> *
  FilterMatchCriterionCompare<std::string,Compare,element>::clone() const
  {
    return new FilterMatchCriterionCompare<std::string,Compare,element>{value_};
  }

  inline bool
  FilterMatchCriterionAnd::operator()(const FilterElementValues &  values) const
  {
    for(auto & criteria : criteria_)
      {
        if(!(*criteria)(values))
          {
            return false;
          }
      }
    return true;
  }

  inline bool
  FilterMatchCriterionOr::operator()(const FilterElementValues &  values) const
  {
    for(auto & criteria : criteria_)
      {
        if((*criteria)(values))
          {
            return true;
          }
      }

    return false;
  }

  inline bool
  FilterMatchCriterionNot::operator()(const FilterElementValues &  values) const
  {
    return !(*pCriterion_)(values);
  }
}
