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
#define  _GLIBCXX_USE_NANOSLEEP
#include "emulator.h"
#include "eventdirector.h"

#include "emane/types.h"
#include "emane/startexception.h"
#include "emane/application/eventgeneratorbuilder.h"

#include <thread>
#include <chrono>
#include <iostream>

namespace EMANE
{
  namespace Application
  {
    class App : public EMANE::Application::Emulator<EventGeneratorBuilder,
                                                    EventDirector,
                                                    EventGeneratorManager>
    {
    public:
      App(const std::string & sName_):
        Emulator(sName_),
        secondsStart_{},
        bNextDay_{}{}

    private:
      std::vector<option> doGetOptions() const
      {
        return {{"starttime", 1, nullptr, 's'},
            {"nextday", 0, nullptr, 'n'}};

      }

      std::string doGetOptString() const
      {
        return "s:n";
      }

      bool doProcessOption(int iOptOpt, const char * pzOptArg)
      {
        switch(iOptOpt)
          {
          case 's':
            {
              unsigned hour, min, sec;
              
              if(sscanf(pzOptArg, "%02u:%02u:%02u", &hour, &min, &sec) == 3)
                {
                  secondsStart_ = std::chrono::seconds{(hour * 3600) + (min * 60) + sec};
                }
              else
                {
                  std::cerr<<"Error in format of start time "
                           <<pzOptArg
                           <<" --starttime HH:MM:SS"<<std::endl; 
                  return false;
                }
            }
            break;

          case 'n':
            bNextDay_ = true;
            break;

          default:
            return false;
          }

        return true;
      }

      virtual std::vector<std::string> doGetOptionsUsage() const
      {
        return 
          {
            "  -s, --starttime TIME           Set the start time HH:MM:SS",
              "  -n, --nextday                  Set the start time to the next day"};
      }


      void doStart() override
      {
        pManager_ = pDirector_->construct();
        
        if(secondsStart_ != std::chrono::seconds::zero())
          {
            std::time_t t = std::time(nullptr);
            
            // current time broken down to mon,day,year,hr,min,sec ...
            struct tm *tm_ptr = std::localtime(&t);
            
            // number of seconds passed so far today
            std::chrono::seconds secondsPassed{tm_ptr->tm_hour * 3600 + tm_ptr->tm_min * 60 + tm_ptr->tm_sec};

            if(bNextDay_)
              {
                // add the remaining seconds left for today to the start time in
                // localtime seconds since midnight
                secondsStart_ += std::chrono::seconds{24 * 60 * 60} - secondsPassed;
              }
            else
              {
                if(secondsStart_ >= secondsPassed)
                  {
                    secondsStart_ -= secondsPassed;
                  }
                else
                  {
                    throw makeException<StartException>("Startime in the past");
                  }
              }

            
            std::this_thread::sleep_for(secondsStart_);
          }
        

        pManager_->start();
        
        pManager_->postStart();
      }


      std::chrono::seconds secondsStart_;
      bool bNextDay_;
    };
  }
}
DECLARE_APPLICATION(EMANE::Application::App,"emaneeventservice");
