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
#include <algorithm>

EMANE::FilterMatchCriterionAnd::FilterMatchCriterionAnd(const std::vector<FilterMatchCriterion *> & criteria):
  criteria_{criteria}{}

EMANE::FilterMatchCriterionAnd *
EMANE::FilterMatchCriterionAnd::create(const std::vector<FilterMatchCriterion *> & criteria)
{
  return new FilterMatchCriterionAnd{criteria};
}

EMANE::FilterMatchCriterionAnd *
EMANE::FilterMatchCriterionAnd::clone() const
{
  std::vector<FilterMatchCriterion *> criteria{};

  std::for_each(criteria_.begin(),
                criteria_.end(),
                [&](const FilterMatchCriterion * p){criteria.push_back(p->clone());});

  return new FilterMatchCriterionAnd{criteria};
}

EMANE::FilterMatchCriterionAnd::~FilterMatchCriterionAnd()
{
  std::for_each(criteria_.begin(),
                criteria_.end(),
                [](const FilterMatchCriterion * p){delete p;});
}

EMANE::FilterMatchCriterionOr::FilterMatchCriterionOr(const std::vector<FilterMatchCriterion *> & criteria):
  criteria_{criteria}{}

EMANE::FilterMatchCriterionOr *
EMANE::FilterMatchCriterionOr::create(const std::vector<FilterMatchCriterion *> & criteria)
{
  return new FilterMatchCriterionOr{criteria};
}

EMANE::FilterMatchCriterionOr *
EMANE::FilterMatchCriterionOr::clone() const
{
  std::vector<FilterMatchCriterion *> criteria{};

  std::for_each(criteria_.begin(),
                criteria_.end(),
                [&](const FilterMatchCriterion * p){criteria.push_back(p->clone());});

  return new FilterMatchCriterionOr{criteria};
}

EMANE::FilterMatchCriterionOr::~FilterMatchCriterionOr()
{
  std::for_each(criteria_.begin(),
                criteria_.end(),
                [](const FilterMatchCriterion * p){delete p;});
}

EMANE::FilterMatchCriterionNot::FilterMatchCriterionNot(FilterMatchCriterion * pCriterion):
  pCriterion_{pCriterion}{}

EMANE::FilterMatchCriterionNot *
EMANE::FilterMatchCriterionNot::create(FilterMatchCriterion  * pCriterion)
{
  return new FilterMatchCriterionNot{pCriterion};
}

EMANE::FilterMatchCriterionNot *
EMANE::FilterMatchCriterionNot::clone() const
{
  return new FilterMatchCriterionNot{pCriterion_->clone()};
}
