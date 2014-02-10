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

#ifndef EMANEAPPLICATIONEMULATOR_HEADER_
#define EMANEAPPLICATIONEMULATOR_HEADER_

#include "main.h"

namespace EMANE
{
  namespace Application
  {
    template<typename Builder, typename Director, typename Manager>
    class Emulator : public Main
    {
    public:
      Emulator(const std::string & sName_):
        Main(sName_){}

    private:
      void doConfigure(const std::string & sXMLURL) override
      {
        pDirector_.reset(new Director{sXMLURL, builder_});
      }
      
      void doStart() override
      {
        pManager_ = pDirector_->construct();
      
        pManager_->start();
    
        pManager_->postStart();
      }
      
      void doStop() override
      {
        pManager_->stop();
      }

      void doDestroy() override
      {
        pManager_->destroy();
      }

    protected:
      Builder builder_;
      std::unique_ptr<Director> pDirector_;
      std::unique_ptr<Manager> pManager_;
    };
  }
}

#endif // EMANEAPPLICATIONEMULATOR_HEADER_
