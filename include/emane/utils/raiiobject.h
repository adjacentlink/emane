/*
 * Copyright (c) 2014-2016 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Adjacent Link LLC nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 *
 * See toplevel COPYING for more information.
 */

#ifndef EMANEUTILSUTILSRAIIOBJECT_HEADER_
#define EMANEUTILSUTILSRAIIOBJECT_HEADER_

#include <functional>
#include <utility>

namespace EMANE
{
  namespace Utils
  {
    template <typename T,typename DeleteFunction>
    class RAIIObject
    {
    public:
      RAIIObject():
        pObject_{}{}

      RAIIObject(T * pObject):
        pObject_{pObject}{}

      RAIIObject(RAIIObject<T,DeleteFunction> && rhs):
        pObject_{}
      {
        reset(rhs.release());
      }

      virtual ~RAIIObject()
      {
        if(pObject_)
          {
            fn_(pObject_);
          }
      }

      RAIIObject & operator=(RAIIObject && rhs)
      {
        reset(rhs.release());
        return *this;
      }

      T * get() const
      {
        return pObject_;
      }

      bool operator!() const
      {
        return !pObject_;
      }

      void reset(T * pObject)
      {
        if(pObject_)
          {
            fn_(pObject_);
          }

        pObject_=pObject;
      }

      void swap(RAIIObject<T,DeleteFunction> & obj)
      {
        std::swap(pObject_,obj.pObject_);
      }

      T * release()
      {
        T * pObject{pObject_};
        pObject_ = nullptr;
        return pObject;
      }

    private:
      T * pObject_;
      DeleteFunction fn_;
    };
  }
}

#endif // EMANEUTILSUTILSRAIIOBJECT_HEADER_
