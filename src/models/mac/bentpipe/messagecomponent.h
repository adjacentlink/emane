/*
 * Copyright (c) 2015,2023 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANE_MODELS_BENTPIPE_MESSAGECOMPONENT_HEADER_
#define EMANE_MODELS_BENTPIPE_MESSAGECOMPONENT_HEADER_

#include "emane/types.h"
#include "emane/utils/vectorio.h"

#include <list>
#include <vector>

namespace EMANE
{
  namespace Models
  {
    namespace BentPipe
    {
      /**
       * @class MessageComponent
       *
       * @brief Holds a message component that may be all or part of
       * an over-the-air frame.
       *
       * Depending on whether aggregation and fragmentation are
       * enabled, the radio model will transmit one or more message
       * components per transmission. A message component, depending
       * on slot (if using TDMA channel access) and message data size,
       * can be one or more entire packets, a portion (fragment) of
       * one or more packets or some combination thereof. A single
       * over-the-air transmission may contain a mixture of both
       * unicast and broadcast message components, where unicast
       * components can be for different destinations.
       */
      class MessageComponent
      {
      public:
        using Data = std::vector<uint8_t>;

        /**
         * Creates a component representing a complement message
         *
         * @param type Type of component
         * @param destination NEM destination
         * @param priority Message priority
         * @param vectorIO Scatter-Gather component data
         */
        MessageComponent(NEMId destination,
                         const Utils::VectorIO & vectorIO);

        /**
         * Creates a component representing a message fragment
         *
         * @param type Type of component
         * @param destination NEM destination
         * @param priority Message priority
         * @param vectorIO Scatter-Gather component data
         * @param fragmentIndex Fragment index
         * @param fragmentOffset Fragment byte offset
         * @param u64FragmentSequence Fragment sequence number
         * @param bMore Flag indicating if more fragment(s) follow
         */
        MessageComponent(NEMId destination,
                         const Utils::VectorIO & vectorIO,
                         size_t fragmentIndex,
                         size_t fragmentOffset,
                         std::uint64_t u64FragmentSequence,
                         bool bMore);

        /**
         * Gets the component data
         *
         * @return Component data reference
         */
        const Data & getData() const;

        /**
         * Gets the destination
         *
         * @return Destination NEM id
         */
        NEMId getDestination() const;

        /**
         * Determines if component is a fragment
         *
         * @return Fragment flag
         */
        bool isFragment() const;

        /**
         * Gets the fragment index
         *
         * @return Fragment index
         */
        size_t getFragmentIndex() const;

        /**
         * Gets the fragment byte offset
         *
         * @return Fragment byte offset
         */
        size_t getFragmentOffset() const;

        /**
         * Gets the fragment sequence number
         *
         * @return Fragment sequence number
         */
        std::uint64_t getFragmentSequence() const;

        /**
         * Determines if a fragment follow this message component
         *
         * @return More fragment flag
         */
        bool isMoreFragments() const;

      private:
        NEMId destination_;
        Data data_;
        size_t fragmentIndex_;
        size_t fragmentOffset_;
        bool bMoreFragments_;
        std::uint64_t u64FragmentSequence_;
      };

      using MessageComponents = std::list<MessageComponent>;
    }
  }
}

#include "messagecomponent.inl"

#endif // EMANE_MODELS_BENTPIPE_MESSAGECOMPONENT_HEADER_
