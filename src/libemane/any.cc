/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/any.h"
#include "emane/utils/parameterconvert.h"

EMANE::Any::Any(std::int64_t i64Value):
  type_{Type::TYPE_INT64},
  i64Value_{i64Value}{}

EMANE::Any::Any(std::uint64_t u64Value):
  type_{Type::TYPE_UINT64},
  u64Value_{u64Value}{}

EMANE::Any::Any(std::int32_t i32Value):
  type_{Type::TYPE_INT32},
  i64Value_{i32Value}{}

EMANE::Any::Any(std::uint32_t u32Value):
  type_{Type::TYPE_UINT32},
  u64Value_{u32Value}{}

EMANE::Any::Any(std::int16_t i16Value):
  type_{Type::TYPE_INT16},
  i64Value_{i16Value}{}

EMANE::Any::Any(std::uint16_t u16Value):
  type_{Type::TYPE_UINT16},
  u64Value_{u16Value}{}

EMANE::Any::Any(std::int8_t i8Value):
  type_{Type::TYPE_INT8},
  i64Value_{i8Value}{}

EMANE::Any::Any(std::uint8_t u8Value):
  type_{Type::TYPE_UINT8},
  u64Value_(u8Value){}

EMANE::Any::Any(float fValue):
  type_{Type::TYPE_FLOAT},
  dValue_{fValue}{}

EMANE::Any::Any(double dValue):
  type_{Type::TYPE_DOUBLE},
  dValue_{dValue}{}

EMANE::Any::Any(const INETAddr & addrValue):
  type_{Type::TYPE_INET_ADDR},
  addrValue_(addrValue){}

EMANE::Any::Any(const bool & bValue):
  type_{Type::TYPE_BOOL},
  u64Value_(bValue){}

EMANE::Any::Any(const char * pzValue):
  type_{Type::TYPE_STRING},
  sValue_{pzValue}{}

EMANE::Any::Any(const std::string & sValue):
  type_{Type::TYPE_STRING},
  sValue_{sValue}{}

EMANE::Any::Any(const Any& rhs)
{
  type_ = rhs.type_;

  if(type_ == Type::TYPE_STRING)
    {
      new (&sValue_) std::string(rhs.sValue_);
    }
  else if(type_ == Type::TYPE_INET_ADDR)
    {
      new (&addrValue_) INETAddr(rhs.addrValue_);
    }
  else
    {
      // copy state  as binary represenation
      memcpy(reinterpret_cast<char *>(this),
             reinterpret_cast<const char *>(&rhs),
             sizeof(Any));
    }
}

EMANE::Any & EMANE::Any::operator=(const Any & rhs)
{
  // if std::string or INETAddr cleanup
  this->~Any();

  type_ = rhs.type_;

  // if std::string or INETAddr use placement new
  // to create object
  if(type_ == Type::TYPE_STRING)
    {
      new (&sValue_) std::string(rhs.sValue_);
    }
  else if(type_ == Type::TYPE_INET_ADDR)
    {
      new (&addrValue_) INETAddr(rhs.addrValue_);
    }
  else
    {
      // copy state  as binary represenation
      memcpy(reinterpret_cast<char *>(this),
             reinterpret_cast<const char *>(&rhs),
             sizeof(Any));
    }

  return *this;
}

EMANE::Any::~Any()
{
  // if std::string or INETAddr cleanup
  if(type_ == Type::TYPE_STRING)
    {
      sValue_.~basic_string<char>();
    }
  else if(type_ == Type::TYPE_INET_ADDR)
    {
      addrValue_.~INETAddr();
    }
}

std::int64_t EMANE::Any::asINT64() const
{
  if(type_ == Type::TYPE_INT64)
    {
      return i64Value_;
    }

  throw AnyException("Not INT64");
}

std::uint64_t EMANE::Any::asUINT64() const
{
  if(type_ == Type::TYPE_UINT64)
    {
      return u64Value_;
    }

  throw AnyException("Not UINT64");
}

std::int32_t EMANE::Any::asINT32() const
{
  if(type_ == Type::TYPE_INT32)
    {
      return i64Value_;
    }

  throw AnyException("Not INT32");
}

std::uint32_t EMANE::Any::asUINT32() const
{
  if(type_ == Type::TYPE_UINT32)
    {
      return u64Value_;
    }

  throw AnyException("Not UINT32");
}

std::int16_t EMANE::Any::asINT16() const
{
  if(type_ == Type::TYPE_INT16)
    {
      return i64Value_;
    }

  throw AnyException("Not INT16");
}

std::uint16_t EMANE::Any::asUINT16() const
{
  if(type_ == Type::TYPE_UINT16)
    {
      return u64Value_;
    }

  throw AnyException("Not UINT16");
}

std::int8_t EMANE::Any::asINT8() const
{
  if(type_ == Type::TYPE_INT8)
    {
      return i64Value_;
    }

  throw AnyException("Not INT8");
}

std::uint8_t EMANE::Any::asUINT8() const
{
  if(type_ == Type::TYPE_UINT8)
    {
      return u64Value_;
    }

  throw AnyException("Not UINT8");
}

float EMANE::Any::asFloat() const
{
  if(type_ == Type::TYPE_FLOAT)
    {
      return dValue_;
    }

  throw AnyException("Not Float");
}

double EMANE::Any::asDouble() const
{
  if(type_ == Type::TYPE_DOUBLE)
    {
      return dValue_;
    }

  throw AnyException("Not Double");
}

EMANE::INETAddr EMANE::Any::asINETAddr() const
{
  if(type_ == Type::TYPE_INET_ADDR)
    {
      return addrValue_;
    }

  throw AnyException("Not INET Addr");
}


bool EMANE::Any::asBool() const
{
  if(type_ == Type::TYPE_BOOL)
    {
      return u64Value_;
    }

  throw AnyException("Not Bool");
}


std::string EMANE::Any::asString() const
{
  if(type_ == Type::TYPE_STRING)
    {
      return sValue_;
    }

  throw AnyException("Not String");
}

EMANE::Any::Type EMANE::Any::getType() const
{
  return type_;
}

std::ostream & operator<<(std::ostream &out, const EMANE::Any & any)
{
  out<<any.toString();
  return out;
}

std::string EMANE::Any::toString() const
{
  switch(getType())
    {
    case EMANE::Any::Type::TYPE_INT64:
      return std::to_string(asINT64());

    case EMANE::Any::Type::TYPE_UINT64:
      return std::to_string(asUINT64());

    case EMANE::Any::Type::TYPE_INT32:
      return std::to_string(asINT32());

    case EMANE::Any::Type::TYPE_UINT32:
      return std::to_string(asUINT32());

    case EMANE::Any::Type::TYPE_INT16:
      return std::to_string(asINT16());

    case EMANE::Any::Type::TYPE_UINT16:
      return std::to_string(asUINT16());

    case EMANE::Any::Type::TYPE_INT8:
      return std::to_string(asINT8());

    case EMANE::Any::Type::TYPE_UINT8:
      return std::to_string(asUINT8());

    case EMANE::Any::Type::TYPE_FLOAT:
      return std::to_string(asFloat());

    case EMANE::Any::Type::TYPE_DOUBLE:
      return std::to_string(asDouble());

    case EMANE::Any::Type::TYPE_INET_ADDR:
      return asINETAddr().str();
      break;

    case EMANE::Any::Type::TYPE_BOOL:
      return std::to_string(asBool());

    case EMANE::Any::Type::TYPE_STRING:
      return asString();

    default:
      break;
    }

  return {};
}

EMANE::Any EMANE::Any::create(std::string sValue, Type type)
{
  try
    {
      switch(type)
        {
        case Any::Type::TYPE_INT64:
          return Any(Utils::ParameterConvert(sValue).toINT64());

        case Any::Type::TYPE_UINT64:
          return Any(Utils::ParameterConvert(sValue).toUINT64());

        case Any::Type::TYPE_INT32:
          return Any(Utils::ParameterConvert(sValue).toINT32());

        case Any::Type::TYPE_UINT32:
          return Any(Utils::ParameterConvert(sValue).toUINT32());

        case Any::Type::TYPE_INT16:
          return Any(Utils::ParameterConvert(sValue).toINT16());

        case Any::Type::TYPE_UINT16:
          return Any(Utils::ParameterConvert(sValue).toUINT16());

        case Any::Type::TYPE_INT8:
          return Any(Utils::ParameterConvert(sValue).toINT8());

        case Any::Type::TYPE_UINT8:
          return Any(Utils::ParameterConvert(sValue).toUINT8());

        case Any::Type::TYPE_FLOAT:
          return Any(Utils::ParameterConvert(sValue).toFloat());

        case Any::Type::TYPE_DOUBLE:
          return Any(Utils::ParameterConvert(sValue).toDouble());

        case Any::Type::TYPE_INET_ADDR:
          return Any(Utils::ParameterConvert(sValue).toINETAddr());

        case Any::Type::TYPE_BOOL:
          return Any(Utils::ParameterConvert(sValue).toBool());

        case Any::Type::TYPE_STRING:
          return Any(sValue.c_str());

        default:
          break;
        }
    }
  catch(Utils::ParameterConvert::ConversionException & exp)
    {
      throw makeException<AnyException>("Conversion failure: %s",exp.what());
    }

  throw AnyException("Invalid type");
}

bool EMANE::Any::operator<=(const EMANE::Any & rhs) const
{
  if(type_ == rhs.type_)
    {
      switch(type_)
        {
        case Any::Type::TYPE_INT64:
        case Any::Type::TYPE_INT32:
        case Any::Type::TYPE_INT16:
        case Any::Type::TYPE_INT8:
          return i64Value_ <= rhs.i64Value_;

        case Any::Type::TYPE_UINT64:
        case Any::Type::TYPE_UINT32:
        case Any::Type::TYPE_UINT16:
        case Any::Type::TYPE_UINT8:
        case Any::Type::TYPE_BOOL:
          return u64Value_ <= rhs.u64Value_;

        case Any::Type::TYPE_FLOAT:
        case Any::Type::TYPE_DOUBLE:
          return dValue_ <= rhs.dValue_;

        case Any::Type::TYPE_STRING:
          return sValue_ <= rhs.sValue_;

        // case Any::Type::TYPE_INET_ADDR:
        //   return (addrValue_ < rhs. addrValue_) ||
        //     (addrValue_ == rhs. addrValue_);

        default:
          break;
        }
    }

  throw EMANE::AnyException("Type mismatch");
}

bool EMANE::Any::operator>=(const EMANE::Any & rhs) const
{
  if(type_ == rhs.type_)
    {
      switch(type_)
        {
        case Any::Type::TYPE_INT64:
        case Any::Type::TYPE_INT32:
        case Any::Type::TYPE_INT16:
        case Any::Type::TYPE_INT8:
          return i64Value_ >=rhs.i64Value_;

        case Any::Type::TYPE_UINT64:
        case Any::Type::TYPE_UINT32:
        case Any::Type::TYPE_UINT16:
        case Any::Type::TYPE_UINT8:
        case Any::Type::TYPE_BOOL:
          return u64Value_ >= rhs.u64Value_;

        case Any::Type::TYPE_FLOAT:
        case Any::Type::TYPE_DOUBLE:
          return dValue_ >= rhs.dValue_;

        case Any::Type::TYPE_STRING:
          return sValue_ >= rhs.sValue_;

        // case Any::Type::TYPE_INET_ADDR:
        //   return (addrValue_ == rhs. addrValue_) ||
        //     !(addrValue_ < rhs. addrValue_);

        default:
          break;
        }
    }

  throw EMANE::AnyException("Type mismatch");
}

bool EMANE::Any::operator<(const EMANE::Any & rhs) const
{
  if(type_ == rhs.type_)
    {
      switch(type_)
        {
        case Any::Type::TYPE_INT64:
        case Any::Type::TYPE_INT32:
        case Any::Type::TYPE_INT16:
        case Any::Type::TYPE_INT8:
          return i64Value_ < rhs.i64Value_;

        case Any::Type::TYPE_UINT64:
        case Any::Type::TYPE_UINT32:
        case Any::Type::TYPE_UINT16:
        case Any::Type::TYPE_UINT8:
        case Any::Type::TYPE_BOOL:
          return u64Value_ < rhs.u64Value_;

        case Any::Type::TYPE_FLOAT:
        case Any::Type::TYPE_DOUBLE:
          return dValue_ < rhs.dValue_;

        case Any::Type::TYPE_STRING:
          return sValue_ < rhs.sValue_;

        // case Any::Type::TYPE_INET_ADDR:
        //   return addrValue_ < rhs. addrValue_;

        default:
          break;
        }
    }

  throw EMANE::AnyException("Type mismatch");
}

bool EMANE::Any::operator>(const EMANE::Any & rhs) const
{
  if(type_ == rhs.type_)
    {
      switch(type_)
        {
        case Any::Type::TYPE_INT64:
        case Any::Type::TYPE_INT32:
        case Any::Type::TYPE_INT16:
        case Any::Type::TYPE_INT8:
          return i64Value_ > rhs.i64Value_;

        case Any::Type::TYPE_UINT64:
        case Any::Type::TYPE_UINT32:
        case Any::Type::TYPE_UINT16:
        case Any::Type::TYPE_UINT8:
        case Any::Type::TYPE_BOOL:
          return u64Value_ > rhs.u64Value_;

        case Any::Type::TYPE_FLOAT:
        case Any::Type::TYPE_DOUBLE:
          return dValue_ > rhs.dValue_;

        case Any::Type::TYPE_STRING:
          return sValue_ > rhs.sValue_;

        // case Any::Type::TYPE_INET_ADDR:
        //   return (addrValue_ != rhs. addrValue_) &&
        //     !(addrValue_ < rhs. addrValue_);

        default:
          break;
        }
    }

  throw EMANE::AnyException("Type mismatch");
}

std::string EMANE::anyTypeAsString(const Any::Type & type)
{
  switch(type)
    {
    case Any::Type::TYPE_INT64:
      return "int64";

    case Any::Type::TYPE_INT32:
      return "int32";

    case Any::Type::TYPE_INT16:
      return "int16";

    case Any::Type::TYPE_INT8:
      return "int8";

    case Any::Type::TYPE_UINT64:
      return "uint64";

    case Any::Type::TYPE_UINT32:
      return "uint32";

    case Any::Type::TYPE_UINT16:
      return "uint16";

    case Any::Type::TYPE_UINT8:
      return "uint8";

    case Any::Type::TYPE_BOOL:
      return "bool";

    case Any::Type::TYPE_FLOAT:
      return "float";

    case Any::Type::TYPE_DOUBLE:
      return "double";

    case Any::Type::TYPE_INET_ADDR:
      return "inetaddr";

    case Any::Type::TYPE_STRING:
      return "string";

    default:
      break;
    }

  throw AnyException("Invalid type");
}

namespace EMANE
{
  template <>
  Any::Type AnyConvertableType<std::uint64_t>::type()
  {
    return Any::Type::TYPE_UINT64;
  }

  template <>
  Any::Type AnyConvertableType<std::int64_t>::type()
  {
    return Any::Type::TYPE_INT64;
  }

  template <>
  Any::Type AnyConvertableType<std::uint32_t>::type()
  {
    return Any::Type::TYPE_UINT32;
  }

  template <>
  Any::Type AnyConvertableType<std::int32_t>::type()
  {
    return Any::Type::TYPE_INT32;
  }

  template <>
  Any::Type AnyConvertableType<std::uint16_t>::type()
  {
    return Any::Type::TYPE_UINT16;
  }

  template <>
  Any::Type AnyConvertableType<std::int16_t>::type()
  {
    return Any::Type::TYPE_INT16;
  }

  template <>
  Any::Type AnyConvertableType<std::uint8_t>::type()
  {
    return Any::Type::TYPE_UINT8;
  }

  template <>
  Any::Type AnyConvertableType<std::int8_t>::type()
  {
    return Any::Type::TYPE_INT8;
  }

  template <>
  Any::Type AnyConvertableType<bool>::type()
  {
    return Any::Type::TYPE_BOOL;
  }

  template <>
  Any::Type AnyConvertableType<float>::type()
  {
    return Any::Type::TYPE_FLOAT;
  }

  template <>
  Any::Type AnyConvertableType<double>::type()
  {
    return Any::Type::TYPE_DOUBLE;
  }

  template <>
  Any::Type AnyConvertableType<const char *>::type()
  {
    return Any::Type::TYPE_STRING;
  }

  template <>
  Any::Type AnyConvertableType<std::string>::type()
  {
    return Any::Type::TYPE_STRING;
  }

  template <>
  Any::Type AnyConvertableType<INETAddr>::type()
  {
    return Any::Type::TYPE_INET_ADDR;
  }
}
