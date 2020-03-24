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

#ifndef EMANEFILTERMATCHCRITERION_HEADER_
#define EMANEFILTERMATCHCRITERION_HEADER_

#include "emane/types.h"
#include <cstdint>
#include <vector>
#include <functional>
#include <memory>

namespace EMANE
{
  enum class FilterElement
    {
     FREQUENCY,
     BANDWIDTH,
     SUBID,
     FILTER_DATA
    };

  struct FilterElementValues
  {
    std::uint64_t u64FrequencyHz_;
    std::uint64_t u64BandwidthHz_;
    std::uint16_t u16SubId_;
    FilterData filterData_;
  };

  class FilterMatchCriterion
  {
  public:
    virtual ~FilterMatchCriterion(){};

    virtual bool operator()(const FilterElementValues & values) const = 0;

    virtual FilterMatchCriterion * clone() const = 0;

  protected:
    FilterMatchCriterion(){}
  };

  template<class T,
           class Compare,
           FilterElement element>
  class FilterMatchCriterionCompare : public FilterMatchCriterion
  {
  public:
    static FilterMatchCriterionCompare<T,Compare,element> * create(T value);

    bool operator()(const FilterElementValues & values) const override;

    FilterMatchCriterionCompare<T,Compare,element> * clone() const override;

  private:
    FilterElement element_;
    T value_;

    FilterMatchCriterionCompare(T value):
      element_{element},
      value_{value}{}

    FilterMatchCriterionCompare(const FilterMatchCriterionCompare &) = delete;

    FilterMatchCriterionCompare &
    operator=(const FilterMatchCriterionCompare &) = delete;
  };

  template<class Compare,
           FilterElement element>
  class FilterMatchCriterionCompare<std::string,Compare,element> : public FilterMatchCriterion
  {
  public:
    static FilterMatchCriterionCompare<std::string,Compare,element> * create(const std::string & value);

    bool operator()(const FilterElementValues & values) const override;

    FilterMatchCriterionCompare<std::string,Compare,element> * clone() const override;

  private:
    FilterElement element_;
    std::string value_;

    FilterMatchCriterionCompare(const std::string & value):
      element_{element},
      value_{value}{}

    FilterMatchCriterionCompare(const FilterMatchCriterionCompare &) = delete;

    FilterMatchCriterionCompare &
    operator=(const FilterMatchCriterionCompare &) = delete;
  };

  class FilterMatchCriterionAnd : public FilterMatchCriterion
  {
  public:
    static FilterMatchCriterionAnd *
    create(const std::vector<FilterMatchCriterion *> & criteria);

    ~FilterMatchCriterionAnd();

    bool operator()(const FilterElementValues &  values) const override;

    FilterMatchCriterionAnd * clone() const override;

  private:
    std::vector<FilterMatchCriterion *> criteria_;

    FilterMatchCriterionAnd(const std::vector<FilterMatchCriterion *> & criteria);

    FilterMatchCriterionAnd(const FilterMatchCriterionAnd &) = delete;

    FilterMatchCriterionAnd &
    operator=(const FilterMatchCriterionAnd &) = delete;
  };

  class FilterMatchCriterionOr : public FilterMatchCriterion
  {
  public:
    static FilterMatchCriterionOr * create(const std::vector<FilterMatchCriterion *> & criteria);

    ~FilterMatchCriterionOr();

    bool operator()(const FilterElementValues &  values) const override;

    FilterMatchCriterionOr * clone() const override;

  private:
    std::vector<FilterMatchCriterion *> criteria_;

    FilterMatchCriterionOr(const std::vector<FilterMatchCriterion *> & criteria);

    FilterMatchCriterionOr(const FilterMatchCriterionOr &) = delete;

    FilterMatchCriterionOr &
    operator=(const FilterMatchCriterionOr &) = delete;
  };

  class FilterMatchCriterionNot : public FilterMatchCriterion
  {
  public:
    static FilterMatchCriterionNot * create(FilterMatchCriterion  * pCriterion);

    bool operator()(const FilterElementValues &  values) const override;

    FilterMatchCriterionNot * clone() const override;

  private:
    std::unique_ptr<FilterMatchCriterion> pCriterion_;

    FilterMatchCriterionNot(FilterMatchCriterion * pCriterion);

    FilterMatchCriterionNot(const FilterMatchCriterionNot &) = delete;

    FilterMatchCriterionNot &
    operator=(const FilterMatchCriterionNot &) = delete;
  };

  using FrequencyFilterElementLess =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::less<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using FrequencyFilterElementLessEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::less_equal<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using FrequencyFilterElementGreater =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::greater<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using FrequencyFilterElementGreaterEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::greater_equal<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using FrequencyFilterElementEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::equal_to<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using FrequencyFilterElementNotEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::not_equal_to<std::uint64_t>,
                                FilterElement::FREQUENCY>;

  using BandwidthFilterElementLess =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::less<std::uint64_t>,
                                FilterElement::BANDWIDTH>;

  using BandwidthFilterElementLessEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::less_equal<std::uint64_t>,
                                FilterElement::BANDWIDTH>;

  using BandwidthFilterElementGreater =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::greater<std::uint64_t>,
                                FilterElement::BANDWIDTH>;


  using BandwidthFilterElementGreaterEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::greater_equal<std::uint64_t>,
                                FilterElement::BANDWIDTH>;

  using BandwidthFilterElementEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::equal_to<std::uint64_t>,
                                FilterElement::BANDWIDTH>;

  using BandwidthFilterElementNotEqual =
    FilterMatchCriterionCompare<std::uint64_t,
                                std::not_equal_to<std::uint64_t>,
                                FilterElement::BANDWIDTH>;


  using SubIdFilterElementLess =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::less<std::uint16_t>,
                                FilterElement::SUBID>;

  using SubIdFilterElementLessEqual =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::less_equal<std::uint16_t>,
                                FilterElement::SUBID>;

  using SubIdFilterElementGreater =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::greater<std::uint16_t>,
                                FilterElement::SUBID>;


  using SubIdFilterElementGreaterEqual =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::greater_equal<std::uint16_t>,
                                FilterElement::SUBID>;

  using SubIdFilterElementEqual =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::equal_to<std::uint16_t>,
                                FilterElement::SUBID>;

  using SubIdFilterElementNotEqual =
    FilterMatchCriterionCompare<std::uint16_t,
                                std::not_equal_to<std::uint16_t>,
                                FilterElement::SUBID>;

  using FilterDataFilterElementEqual =
    FilterMatchCriterionCompare<FilterData,
                                std::equal_to<FilterData>,
                                FilterElement::FILTER_DATA>;

  using FilterDataFilterElementNotEqual =
    FilterMatchCriterionCompare<FilterData,
                                std::not_equal_to<FilterData>,
                                FilterElement::FILTER_DATA>;
};

#include <emane/filtermatchcriterion.inl>

#endif // EMANEFILTERMATCHCRITERION_HEADER_
