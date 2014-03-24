/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSRECVCANCELABLE_HEADER_
#define EMANEUTILSRECVCANCELABLE_HEADER_

#include <ace/OS_NS_sys_select.h>

namespace EMANE
{
  namespace Utils
  {
    /**
     * Utility function for turning a blocking read call into a cancellation point in OSX
     *
     * @param ref ACE socket reference
     * @param buf Output buffer
     * @param n Output buffer size
     * @param addr Reference to hold sender's address
     * @param flags socket read flags
     *
     * @return number bytes read
     */
    template <typename T>
    ssize_t recvCancelable(T & ref, void * buf, size_t n, ACE_Addr &addr, int flags = 0)
    {
#ifdef __APPLE__
    
      // on OSX the recv system call is not cancelable.  So threads
      // blocking on a recv will not be cleaned up on an ACE_OS::thr_cancel
      // and will be unjoinable with an ACE_OS::thr_join.  Although not
      // listed as a cancellation point, select is cancelable - at least during 
      // testing on OSX 10.5.7 (Leopard)...

      fd_set rfds;

      ACE_HANDLE handle = ref.get_handle();
    
      FD_ZERO(&rfds);

      FD_SET(handle,&rfds);

      if(ACE_OS::select(handle+1,&rfds))
        {
          return ref.recv(buf,n,addr,flags);
        }

      return -1;

#else

      return ref.recv(buf,n,addr,flags);

#endif
    }
  }
}

#endif // EMANEUTILSRECVCANCELABLE_HEADER_
