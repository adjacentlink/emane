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

#ifndef EMANEUTILSSPAWNMEMBERFUNC_HEADER_
#define EMANEUTILSSPAWNMEMBERFUNC_HEADER_

#include <ace/Thread.h>
#include <memory>

namespace
{
  template <class T>
  class Func
  {
  public:
    T & r_;
    ACE_THR_FUNC_RETURN (T::*f_)();
    Func(T & r, ACE_THR_FUNC_RETURN (T::*f)()):r_(r),f_(f){};
  };

  template <class T>
  ACE_THR_FUNC_RETURN start_routine (void * arg)
  {
    std::unique_ptr<Func<T>> p(reinterpret_cast<Func<T> *>(arg));
    return (p->r_.*(p->f_))();
  }
}

namespace EMANE
{
  namespace Utils
  {
    /**
     * Create a new thread, which executes @a member function of class @a T.
     *
     * @return Unique group id used to control other threads added to the same group
     *  or -1 on failure
     *
     * @note See ACE Thread_Manager documentation
     */
    template <class T>
    int spawn(T & ref,
              ACE_THR_FUNC_RETURN (T::*method)(),
              ACE_thread_t *t_thread  = 0,
              ACE_hthread_t *t_handle = 0,
              long flags              = THR_NEW_LWP|THR_JOINABLE|THR_INHERIT_SCHED,
              long priority           = ACE_DEFAULT_THREAD_PRIORITY,
              ACE_Base_Thread_Adapter *  thread_adapter = 0,
              void *stack             = 0,
              size_t stack_size       = ACE_DEFAULT_THREAD_STACKSIZE,
              const char** thr_name   = 0)
    {
      return  ACE_OS::thr_create(&start_routine<T>, 
                                 new Func<T>(ref,method),
                                 flags,
                                 t_thread,
                                 t_handle,
                                 priority,
                                 stack,
                                 stack_size,
                                 thread_adapter,
                                 thr_name);      
    }
  }
}

#endif //EMANEUTILSSPAWNMEMBERFUNC_HEADER_
