/*
 * Copyright (c) 2017 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEEVENTSFADINGSELECTION_HEADER_
#define EMANEEVENTSFADINGSELECTION_HEADER_

#include "emane/types.h"
#include "emane/events/fadingmodel.h"

#include <list>

namespace EMANE
{
  namespace Events
  {
    /**
     * @class FadingProfile
     *
     * @brief Holds NEM Id and fading model
     *
     * @note Instances are immutable
     */
    class FadingSelection
    {
    public:
      /**
       * Creates an FadingSelection instance with a NEM Id and fading
       * model.
       */
      FadingSelection();

      /**
       * Creates an FadingSelection instance
       *
       * @param nemId Id of NEM
       * @param fadingModel Fading model
       */
      FadingSelection(NEMId nemId,
                      FadingModel fadingModel);

      /**
       * Gets the NEM Id
       *
       * @return NEM Id
       */
      NEMId getNEMId() const;

      /**
       * Gets the fading model
       *
       * @return fading model
       */
      FadingModel getFadingModel() const;

    private:
      NEMId nemId_;
      FadingModel fadingModel_;
    };

    using FadingSelections = std::list<FadingSelection>;
  }
}

#include "emane/events/fadingselection.inl"

#endif // EMANEEVENTSFADINGSELECTION_HEADER_
