/*
 * Copyright (c) 2019-2020 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/controls/spectrumfilteraddcontrolmessage.h"

class EMANE::Controls::SpectrumFilterAddControlMessage::Implementation
{
public:
  Implementation(FilterIndex filterIndex,
                 std::uint64_t u64FrequencyHz,
                 std::uint64_t u64BandwidthHz,
                 std::uint64_t u64SubBandBinSizeHz,
                 const FilterMatchCriterion * pFilterMatchCriterion):
    filterIndex_{filterIndex},
    u64FrequencyHz_{u64FrequencyHz},
    u64BandwidthHz_{u64BandwidthHz},
    u64SubBandBinSizeHz_{u64SubBandBinSizeHz},
    pFilterMatchCriterion_{pFilterMatchCriterion}{}

  Implementation(const Implementation & impl):
    filterIndex_{impl.filterIndex_},
    u64FrequencyHz_{impl.u64FrequencyHz_},
    u64BandwidthHz_{impl.u64BandwidthHz_},
    u64SubBandBinSizeHz_{impl.u64SubBandBinSizeHz_},
    pFilterMatchCriterion_{impl.pFilterMatchCriterion_->clone()}{}

  FilterIndex getFilterIndex() const
  {
    return filterIndex_;
  }

  std::uint64_t getBandwidthHz() const
  {
    return u64BandwidthHz_;
  }

  std::uint64_t getFrequencyHz() const
  {
    return u64FrequencyHz_;
  }

  std::size_t getSubBandBinSizeHz() const
  {
    return u64SubBandBinSizeHz_;
  }

  const FilterMatchCriterion * getFilterMatchCriterion() const
  {
    return pFilterMatchCriterion_.get();
  }

private:
  const FilterIndex filterIndex_;
  const std::uint64_t u64FrequencyHz_;
  const std::uint64_t u64BandwidthHz_;
  const std::uint64_t u64SubBandBinSizeHz_;
  std::unique_ptr<const FilterMatchCriterion> pFilterMatchCriterion_;
};

EMANE::Controls::SpectrumFilterAddControlMessage::
SpectrumFilterAddControlMessage(const SpectrumFilterAddControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}

EMANE::Controls::SpectrumFilterAddControlMessage::
SpectrumFilterAddControlMessage(FilterIndex filterIndex,
                                std::uint64_t u64FrequencyHz,
                                std::uint64_t u64BandwidthHz,
                                std::uint64_t u64SubBandBinSizeHz,
                                FilterMatchCriterion * pFilterMatchCriterion):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{filterIndex,
                            u64FrequencyHz,
                            u64BandwidthHz,
                            u64SubBandBinSizeHz,
                            pFilterMatchCriterion}}{}

EMANE::Controls::SpectrumFilterAddControlMessage::~SpectrumFilterAddControlMessage(){}


EMANE::Controls::SpectrumFilterAddControlMessage *
EMANE::Controls::SpectrumFilterAddControlMessage::create(FilterIndex filterIndex,
                                                         std::uint64_t u64FrequencyHz,
                                                         std::uint64_t u64BandwidthHz,
                                                         std::uint64_t u64SubBandBinSizeHz,
                                                         FilterMatchCriterion * pFilterMatchCriterion)
{
  return new SpectrumFilterAddControlMessage{filterIndex,
                                               u64FrequencyHz,
                                               u64BandwidthHz,
                                               u64SubBandBinSizeHz,
                                               pFilterMatchCriterion};

}

std::uint64_t
EMANE::Controls::SpectrumFilterAddControlMessage::getBandwidthHz() const
{
  return pImpl_->getBandwidthHz();
}

std::uint64_t
EMANE::Controls::SpectrumFilterAddControlMessage::getFrequencyHz() const
{
  return pImpl_->getFrequencyHz();
}

const EMANE::FilterMatchCriterion *
EMANE::Controls::SpectrumFilterAddControlMessage::getFilterMatchCriterion() const
{
  return pImpl_->getFilterMatchCriterion();
}

EMANE::FilterIndex
EMANE::Controls::SpectrumFilterAddControlMessage::getFilterIndex() const
{
  return pImpl_->getFilterIndex();
}

std::uint64_t
EMANE::Controls::SpectrumFilterAddControlMessage::getSubBandBinSizeHz() const
{
  return pImpl_->getSubBandBinSizeHz();
}

EMANE::Controls::SpectrumFilterAddControlMessage *
EMANE::Controls::SpectrumFilterAddControlMessage::clone() const
{
  return new SpectrumFilterAddControlMessage{*this};
}
