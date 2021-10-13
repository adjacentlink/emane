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

#include "emane/controls/spectrumfilterremovecontrolmessage.h"

class EMANE::Controls::SpectrumFilterRemoveControlMessage::Implementation
{
public:
  Implementation(FilterIndex filterIndex,
                 AntennaIndex antennaIndex):
    filterIndex_{filterIndex},
    antennaIndex_{antennaIndex}{}

  Implementation(const Implementation & impl):
    filterIndex_{impl.filterIndex_},
    antennaIndex_{impl.antennaIndex_}{}

  FilterIndex getFilterIndex() const
  {
    return filterIndex_;
  }

  AntennaIndex getAntennaIndex() const
  {
    return antennaIndex_;
  }

private:
  const FilterIndex filterIndex_;
  const AntennaIndex antennaIndex_;
};

EMANE::Controls::SpectrumFilterRemoveControlMessage::
SpectrumFilterRemoveControlMessage(const SpectrumFilterRemoveControlMessage & msg):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{*msg.pImpl_}}
{}

EMANE::Controls::SpectrumFilterRemoveControlMessage::
SpectrumFilterRemoveControlMessage(FilterIndex filterIndex,
                                   AntennaIndex antennaIndex):
  ControlMessage{IDENTIFIER},
  pImpl_{new Implementation{filterIndex,antennaIndex}}{}

EMANE::Controls::SpectrumFilterRemoveControlMessage::~SpectrumFilterRemoveControlMessage(){}


EMANE::Controls::SpectrumFilterRemoveControlMessage *
EMANE::Controls::SpectrumFilterRemoveControlMessage::create(FilterIndex filterIndex,
                                                            AntennaIndex antennaIndex)
{
  return new SpectrumFilterRemoveControlMessage{filterIndex,antennaIndex};
}

EMANE::Controls::SpectrumFilterRemoveControlMessage *
EMANE::Controls::SpectrumFilterRemoveControlMessage::create(FilterIndex filterIndex)
{
  return new SpectrumFilterRemoveControlMessage{filterIndex,DEFAULT_ANTENNA_INDEX};
}

EMANE::FilterIndex
EMANE::Controls::SpectrumFilterRemoveControlMessage::getFilterIndex() const
{
  return pImpl_->getFilterIndex();
}

EMANE::AntennaIndex
EMANE::Controls::SpectrumFilterRemoveControlMessage::getAntennaIndex() const
{
  return pImpl_->getAntennaIndex();
}

EMANE::Controls::SpectrumFilterRemoveControlMessage *
EMANE::Controls::SpectrumFilterRemoveControlMessage::clone() const
{
  return new SpectrumFilterRemoveControlMessage{*this};
}
