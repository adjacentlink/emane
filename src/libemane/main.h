/*
 * Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANEAPPLICATIONMAIN_HEADER_
#define EMANEAPPLICATIONMAIN_HEADER_

#include <string>
#include <memory>
#include <vector>
#include <uuid.h>
#include <getopt.h>

#include "emane/application/logger.h"


namespace EMANE
{
  namespace Application
  {
    class Main
    {
    public:
      virtual ~Main();
      
      int main(int argc, char * argv[]);
      
    protected:
      Main(const std::string & sName);
      
      virtual const uuid_t & getUUID() const;
      
      virtual void doConfigure(const std::string &) = 0;
      
      virtual void doStart() = 0;
      
      virtual void doStop() = 0;

      virtual void doDestroy() = 0;
      
      virtual std::vector<option> doGetOptions() const
      {
        return {};
      }

      virtual std::vector<std::string> doGetOptionsUsage() const
      {
        return {};
      }

      virtual std::string doGetOptString() const
      {
        return {};
      }

      virtual bool doProcessOption(int iOptOpt, const char * pzOptArg)
      {
        (void) iOptOpt;
        (void) pzOptArg;
        return false;
      }

    private:
      std::string sName_;
      uuid_t uuid_;
    };
  }
}

#define DECLARE_APPLICATION(X,name)                      \
  int main(int argc, char * argv[])                      \
  {                                                             \
    std::unique_ptr<EMANE::Application::Main> p{new X{(name)}}; \
    return p->main(argc,argv);                                  \
  }                                                      

#endif //EMANEAPPLICATIONMAIN_HEADER_


