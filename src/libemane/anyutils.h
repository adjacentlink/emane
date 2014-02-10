/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONTROLPORTANYUTILS_HEADER_
#define EMANECONTROLPORTANYUTILS_HEADER_

#include "remotecontrolportapi.pb.h"
#include "emane/any.h"

namespace EMANE
{
  namespace ControlPort
  {
    inline
    Any::Type toAnyType(EMANERemoteControlPortAPI::Any::AnyType type)
    {
      switch(type)
        {
        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT8:
          return Any::Type::TYPE_INT8;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT8:
          return Any::Type::TYPE_UINT8;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT16:
          return Any::Type::TYPE_INT16;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT16:
          return Any::Type::TYPE_UINT16;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT32:
          return Any::Type::TYPE_INT32;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT32:
          return Any::Type::TYPE_UINT32;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT64:
          return Any::Type::TYPE_INT64;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT64:
          return Any::Type::TYPE_UINT64;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_FLOAT:
          return Any::Type::TYPE_FLOAT;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_DOUBLE:
          return Any::Type::TYPE_DOUBLE;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_STRING:
          return Any::Type::TYPE_STRING;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_BOOLEAN:
          return Any::Type::TYPE_BOOL;

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INETADDR:
          return Any::Type::TYPE_INET_ADDR;
        }

      throw AnyException("unknown type");
    }

    inline
    Any toAny(EMANERemoteControlPortAPI::Any any)
    {
      switch(any.type())
        {
        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT8:
          return Any{static_cast<std::int8_t>(any.i32value())};
        
        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT8:
          return Any{static_cast<std::uint8_t>(any.u32value())};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT16:
          return Any{static_cast<std::int16_t>(any.i32value())};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT16:
          return Any{static_cast<std::uint16_t>(any.u32value())};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT32:
          return Any{static_cast<std::int32_t>(any.i32value())};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT32:
          return Any{static_cast<std::uint32_t>(any.u32value())};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INT64:
          return Any{any.i64value()};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT64:
          return Any{any.u64value()};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_FLOAT:
          return Any{any.fvalue()};
         
        case EMANERemoteControlPortAPI::Any::TYPE_ANY_DOUBLE:
          return Any{any.dvalue()};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_STRING:
          return Any{any.svalue()};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_BOOLEAN:
          return EMANE::Any{any.bvalue()};

        case EMANERemoteControlPortAPI::Any::TYPE_ANY_INETADDR:
          return EMANE::Any{ACE_INET_Addr(any.svalue().c_str())};
        }

      throw AnyException("unknown type");
    }

    inline
    void convertToAny(EMANERemoteControlPortAPI::Any * pAny,
                      const EMANE::Any & any)
    {
      switch(any.getType())
        {
        case Any::Type::TYPE_INT64:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_INT64);
          pAny->set_i64value(any.asINT64());
          break;
        
        case Any::Type::TYPE_UINT64:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT64);
          pAny->set_u64value(any.asUINT64());
          break;

        case Any::Type::TYPE_INT32:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_INT32);
          pAny->set_i32value(any.asINT32());
          break;

        case Any::Type::TYPE_UINT32:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT32);
          pAny->set_u32value(any.asUINT32());
          break;

        case Any::Type::TYPE_INT16:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_INT16);
          pAny->set_i32value(any.asINT16());
          break;

        case Any::Type::TYPE_UINT16:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT16);
          pAny->set_u32value(any.asUINT16());
          break;

        case Any::Type::TYPE_INT8:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_INT8);
          pAny->set_i32value(any.asINT8());
          break;

        case Any::Type::TYPE_UINT8:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_UINT8);
          pAny->set_u32value(any.asUINT8());
          break;

        case Any::Type::TYPE_FLOAT:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_FLOAT);
          pAny->set_fvalue(any.asFloat());
          break;

        case Any::Type::TYPE_DOUBLE:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_DOUBLE);
          pAny->set_dvalue(any.asDouble());
          break;

        case Any::Type::TYPE_INET_ADDR:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_INETADDR);
          // slight cheat - send INET Addrs using their string
          // representation
          {
            char buf[128];
          
            any.asINETAddr().addr_to_string(buf,
                                            sizeof(buf));
          
            pAny->set_svalue(buf);
          }
          break;

        case Any::Type::TYPE_BOOL:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_BOOLEAN);
          pAny->set_bvalue(any.asBool());
          break;

        case Any::Type::TYPE_STRING:
          pAny->set_type(EMANERemoteControlPortAPI::Any::TYPE_ANY_STRING);
          pAny->set_svalue(any.asString());
          break;

        default:
          throw AnyException("unknown type");
        }
    }
  }
}



#endif // EMANECONTROLPORTANYUTILS_HEADER_
