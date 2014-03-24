/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSPARAMETERCONVERT_HEADER_
#define EMANEUTILSPARAMETERCONVERT_HEADER_

#include "emane/exception.h"

#include <string>
#include <cstdint>
#include <limits>

#include <ace/INET_Addr.h>

namespace EMANE
{
  namespace Utils
  {
    /**
     * @class ParameterConvert
     *
     * @brief Parameter conversion class with range checks
     *
     */
    class ParameterConvert
    {
    public:

      /**
       * @class ConversionException
       *
       * @brief Parameter conversion exception class
       *
       */
      class ConversionException : public EMANE::Exception
      {
      public:
        ConversionException(const std::string & sDescription):
          Exception("ConvertionException",sDescription){}
      
        ~ConversionException() throw(){}
      };

      /**
       * Creates a ParameterConvert instance
       *
       * @param sParameter Value as a string
       */
      ParameterConvert(const std::string & sParameter);
    
      /**
       * Destroys an instance
       */
      ~ParameterConvert();
    
      /**
       * Convert parameter string to an std::int64_t
       *
       * @param i64Min Minimum value in range
       * @param i64Max Maximum value in range
       *
       * @return std::int64_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::int64_t toINT64(std::int64_t i64Min = std::numeric_limits<std::int64_t>::min(),
                           std::int64_t i64Max = std::numeric_limits<std::int64_t>::max()) const;

      /**
       * Convert parameter string to an std::uint64_t
       *
       * @param u64Min Minimum value in range
       * @param u64Max Maximum value in range
       *
       * @return std::uint64_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::uint64_t toUINT64(std::uint64_t u64Min = std::numeric_limits<std::uint64_t>::min(),
                             std::uint64_t u64Max = std::numeric_limits<std::uint64_t>::max()) const;

    
      /**
       * Convert parameter string to an std::int32_t
       *
       * @param i32Min Minimum value in range
       * @param i32Max Maximum value in range
       *
       * @return std::int32_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::int32_t toINT32(std::int32_t i32Min = std::numeric_limits<std::int32_t>::min(),
                           std::int32_t i32Max = std::numeric_limits<std::int32_t>::max()) const;

      /**
       * Convert parameter string to an std::uint32_t
       *
       * @param u32Min Minimum value in range
       * @param u32Max Maximum value in range
       *
       * @return std::uint32_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::uint32_t toUINT32(std::uint32_t u32Min = std::numeric_limits<std::uint32_t>::min(),
                             std::uint32_t u32Max = std::numeric_limits<std::uint32_t>::max()) const;

      /**
       * Convert parameter string to an std::int16_t
       *
       * @param i16Min Minimum value in range
       * @param i16Max Maximum value in range
       *
       * @return std::int16_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::int16_t toINT16(std::int16_t i16Min = std::numeric_limits<std::int16_t>::min(),
                           std::int16_t i16Max = std::numeric_limits<std::int16_t>::max()) const;
    
      /**
       * Convert parameter string to an std::uint16_t
       *
       * @param u16Min Minimum value in range
       * @param u16Max Maximum value in range
       *
       * @return std::uint16_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::uint16_t toUINT16(std::uint16_t u16Min = std::numeric_limits<std::uint16_t>::min(),
                             std::uint16_t u16Max = std::numeric_limits<std::uint16_t>::max()) const;

      /**
       * Convert parameter string to an std::int8_t
       *
       * @param i8Min Minimum value in range
       * @param i8Max Maximum value in range
       *
       * @return std::int8_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::int8_t toINT8(std::int8_t i8Min = std::numeric_limits<std::int8_t>::min(),
                         std::int8_t i8Max = std::numeric_limits<std::int8_t>::max()) const;

      /**
       * Convert parameter string to an  std::uint8_t
       *
       * @param u8Min Minimum value in range
       * @param u8Max Maximum value in range
       *
       * @return  std::uint8_t value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      std::uint8_t toUINT8(std::uint8_t u8Min = std::numeric_limits<std::uint8_t>::min(),
                           std::uint8_t u8Max = std::numeric_limits<std::uint8_t>::max()) const;

      /**
       * Convert parameter string to a float
       *
       * @param fMin Minimum value in range
       * @param fMax Maximum value in range
       *
       * @return float value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      float toFloat(float fMin = std::numeric_limits<float>::lowest(),
                    float fMax = std::numeric_limits<float>::max()) const;

      /**
       * Convert parameter string to a double
       *
       * @param dMin Minimum value in range
       * @param dMax Maximum value in range
       *
       * @return double value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      double toDouble(double dMin = std::numeric_limits<double>::lowest(),
                      double dMax = std::numeric_limits<double>::max()) const;

      /**
       * Convert parameter string to an ACE_INET_Addr
       *
       * @return ACE_INET_Addr value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      ACE_INET_Addr toINETAddr() const;

      /**
       * Convert parameter string to an bool
       *
       * @return bool value
       *
       * @exception ConversionException Thrown when an error is encountered during
       * conversion either to input format or out of range value.
       */
      bool toBool() const;

    private:
      std::string sParameter_;
    };
  }
}

#include "parameterconvert.inl"

#endif //EMANEUTILSPARAMETERCONVERT_HEADER_
