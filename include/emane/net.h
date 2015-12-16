/*
 * Copyright (c) 2008-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENET_HEADER_
#define EMANENET_HEADER_

#include <cstdlib>
#include <cstdio>

namespace EMANE
{
#ifdef HTONL
#undef HTONL
#endif
#ifdef HTONS
#undef HTONS
#endif
#ifdef NTOHL
#undef NTOHL
#endif
#ifdef NTOHS
#undef NTOHS
#endif 
#ifdef NTOHLL
#undef NTOHLL
#endif 
#ifdef HTONLL
#undef HTONLL
#endif 

#define SWAP_16(x)                              \
  ((((x) & 0XFF00) >> 8)) |                     \
  (((x) & 0X00FF) << 8)

#define SWAP_32(x)                              \
  ((((x) & 0XFF000000UL) >> 24) |               \
   (((x) & 0X00FF0000UL) >>  8) |               \
   (((x) & 0X0000FF00UL) <<  8) |               \
   (((x) & 0X000000FFUL) << 24))

#define SWAP_64(x)                              \
  ((((x) & 0XFF00000000000000ULL) >> 56) |      \
   (((x) & 0X00FF000000000000ULL) >> 40) |      \
   (((x) & 0X0000FF0000000000ULL) >> 24) |      \
   (((x) & 0X000000FF00000000ULL) >> 8)  |      \
   (((x) & 0X00000000FF000000ULL) << 8)  |      \
   (((x) & 0X0000000000FF0000ULL) << 24) |      \
   (((x) & 0X000000000000FF00ULL) << 40) |      \
   (((x) & 0X00000000000000FFULL) << 56))


  constexpr std::uint64_t HTONLL(std::uint64_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_64(x);
#else
# error "need to specify endian type"
#endif
  }

  constexpr std::uint64_t NTOHLL(std::uint64_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_64(x);
#else
# error "need to specify endian type"
#endif
  }

  constexpr std::uint32_t HTONL(std::uint32_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_32(x);
#else
# error "need to specify endian type"
#endif
  }

  constexpr std::uint32_t NTOHL(std::uint32_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_32(x);
#else
# error "need to specify endian type"
#endif
  }

  constexpr std::uint16_t HTONS(std::uint16_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_16(x);
#else
# error "need to specify endian type"
#endif
  }

  constexpr std::uint16_t NTOHS(std::uint16_t x)
  {
#if __BYTE_ORDER == __BIG_ENDIAN
    return x;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    return SWAP_16(x);
#else
# error "need to specify endian type"
#endif
  }
}

#endif //EMANE_NET_HEADER_
