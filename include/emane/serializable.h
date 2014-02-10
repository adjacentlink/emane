/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANESERIALIZABLE_HEADER_
#define EMANESERIALIZABLE_HEADER_

#include "emane/serializationexception.h"

#include <string>

namespace EMANE
{
  using Serialization = std::string;
  
  /**
   * @class Serializable
   *
   * @brief The Serializable interface provides the API used
   * to serialize headers, control messages and events.
   */
  class Serializable
  {
  public:
    virtual ~Serializable(){}
    
    /**
     * Gets the serialized form of the object
     *
     * @return serialization
     *
     * @throw SerializationException
     *
     * @note Most objects use Google Protocol Buffers to
     * perform serialization.  This is why the Serialization type
     * is a std::string. The serialized data is binary, non-printable
     * data. The std::string just provides a simple API.
     *
     * @note https://developers.google.com/protocol-buffers/
     */
    virtual Serialization serialize() const
    {
      throw SerializationException{"Not implemented"};
    }
    
  protected:
    Serializable() = default;
  };
}

#endif // EMANESERIALIZABLE_HEADER_
