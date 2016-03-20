/*
 * Copyright (c) 2015-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009,2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include <sstream>
#include <cstdlib>
#include <cstring>

namespace
{
  std::string scaleNumericalStringRepresentation(const std::string & sValue)
  {
    std::string sTmpParameter(sValue);

    std::uint8_t u8PowerOf10 = 0;

    switch(*(sValue.end() - 1))
      {
      case 'G':
        sTmpParameter.assign(sValue,0,sValue.size() - 1);
        u8PowerOf10 = 9;
        break;

      case 'M':
        sTmpParameter.assign(sValue,0,sValue.size() - 1);
        u8PowerOf10 = 6;
        break;

      case 'K':
        sTmpParameter.assign(sValue,0,sValue.size() - 1);
        u8PowerOf10 = 3;
        break;
      }

    if(u8PowerOf10 != 0)
      {
        // strip any leading whitespace
        std::string::size_type notWhiteSpace{sTmpParameter.find_first_not_of(" \t")};

        if(notWhiteSpace != std::string::npos)
          {
            sTmpParameter = sTmpParameter.substr(notWhiteSpace,std::string::npos);
          }

        // location of decimal point, if exists
        std::string::size_type indexPoint =  sTmpParameter.find(".",0);

        if(indexPoint != std::string::npos)
          {
            std::string::size_type numberOfDigitsAfterPoint =
              sTmpParameter.size() - indexPoint - 1;

            if(numberOfDigitsAfterPoint > u8PowerOf10)
              {
                // need to move the decimal point, enough digits are present
                sTmpParameter.insert(indexPoint + u8PowerOf10,".");
              }
            else
              {
                // need to append 0s
                sTmpParameter.append(u8PowerOf10 - numberOfDigitsAfterPoint,'0');
              }

            // remove original decimal point
            sTmpParameter.erase(indexPoint,1);
          }
        else
          {
            // need to append 0s
            sTmpParameter.append(u8PowerOf10,'0');
          }

        // strip any leading zeros but be mindful of sign
        std::string sSign{};

        if(sTmpParameter.front() == '+' ||
           sTmpParameter.front() == '-')
          {
            sSign = sTmpParameter.front();
          }

        std::string::size_type not0Index{sTmpParameter.find_first_not_of("0",sSign.size())};

        if(not0Index != sSign.size())
          {
            sTmpParameter = sSign + sTmpParameter.substr(not0Index,std::string::npos);
          }
      }

    return sTmpParameter;
  }
}

inline
EMANE::Utils::ParameterConvert::ParameterConvert(const std::string & sParameter):
  sParameter_{sParameter}{}

inline
EMANE::Utils::ParameterConvert::~ParameterConvert(){}

inline
std::int64_t EMANE::Utils::ParameterConvert::toINT64(std::int64_t i64Min, std::int64_t i64Max) const
{
  long long llValue = 0;

  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));

      char * pEnd = 0;

      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      llValue =  std::strtoll(sTmpParameter.c_str(),&pEnd,0);

      if(errno == ERANGE ||
         llValue < i64Min ||
         llValue > i64Max)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<i64Min<<","<<i64Max<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }

  return llValue;
}

inline
std::uint64_t EMANE::Utils::ParameterConvert::toUINT64(std::uint64_t u64Min, std::uint64_t u64Max) const
{
  unsigned long long ullValue = 0;

  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));

      char * pEnd = 0;

      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      ullValue =  strtoull(sTmpParameter.c_str(),&pEnd,0);

      if(errno == ERANGE ||
         ullValue < u64Min ||
         ullValue > u64Max)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<u64Min<<","<<u64Max<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }

  return ullValue;
}

inline
std::int32_t EMANE::Utils::ParameterConvert::toINT32(std::int32_t i32Min, std::int32_t i32Max) const
{
  return static_cast<std::int32_t>(toINT64(i32Min,i32Max));
}

inline
std::uint32_t EMANE::Utils::ParameterConvert::toUINT32(std::uint32_t u32Min , std::uint32_t u32Max) const
{
  return static_cast<std::uint32_t>(toUINT64(u32Min,u32Max));
}

inline
std::int16_t EMANE::Utils::ParameterConvert::toINT16(std::int16_t i16Min, std::int16_t i16Max) const
{
  return static_cast<std::int16_t>(toINT64(i16Min,i16Max));
}

inline
std::uint16_t EMANE::Utils::ParameterConvert::toUINT16(std::uint16_t u16Min , std::uint16_t u16Max) const
{
  return static_cast<std::uint16_t>(toUINT64(u16Min,u16Max));
}

inline
std::int8_t EMANE::Utils::ParameterConvert::toINT8(std::int8_t i8Min, std::int8_t i8Max) const
{
  return static_cast<std::int8_t>(toINT64(i8Min,i8Max));
}

inline
std::uint8_t EMANE::Utils::ParameterConvert::toUINT8(std::uint8_t u8Min , std::uint8_t u8Max) const
{
  return static_cast<std::uint8_t>(toUINT64(u8Min,u8Max));
}

inline
bool EMANE::Utils::ParameterConvert::toBool() const
{
  if(!strcasecmp(sParameter_.c_str(),"on")   ||
     !strcasecmp(sParameter_.c_str(),"yes")  ||
     !strcasecmp(sParameter_.c_str(),"true") ||
     !strcasecmp(sParameter_.c_str(),"1"))
    {
      return true;
    }
  else if(!strcasecmp(sParameter_.c_str(),"off")   ||
          !strcasecmp(sParameter_.c_str(),"no")  ||
          !strcasecmp(sParameter_.c_str(),"false") ||
          !strcasecmp(sParameter_.c_str(),"0"))
    {
      return false;
    }
  else
    {
      std::stringstream sstream;
      sstream<<"'"<<sParameter_<<"' invalid boolean conversion"<<std::ends;
      throw ConversionException(sstream.str());
    }
}

inline
EMANE::INETAddr EMANE::Utils::ParameterConvert::toINETAddr() const
{
  INETAddr addr;

  try
    {
      addr.set(sParameter_);
    }
  catch(...)
    {
      std::stringstream sstream;
      sstream<<"'"<<sParameter_<<"' Invalid IP Address"<<std::ends;
      throw ConversionException(sstream.str());
    }

  return addr;
}

inline
double EMANE::Utils::ParameterConvert::toDouble(double dMin, double dMax) const
{
  double dValue = 0;

  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));

      char * pEnd = 0;

      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      dValue =  std::strtod(sTmpParameter.c_str(),&pEnd);

      if(errno == ERANGE ||
         dValue < dMin ||
         dValue > dMax)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<dMin<<","<<dMax<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }

  return dValue;
}

inline
float EMANE::Utils::ParameterConvert::toFloat(float fMin, float fMax) const
{
  return static_cast<float>(toDouble(fMin,fMax));
}
