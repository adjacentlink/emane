/*
 * Copyright (c) 2014,2016,2025 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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

#include "main.h"
#include "emane/application/libemane.h"
#include "emane/application/logger.h"
#include "emane/application/activitycoordinator.h"
#include "emane/exception.h"
#include "emane/utils/parameterconvert.h"

#include <iostream>
#include <fstream>
#include <mutex>
#include <functional>

#include <getopt.h>
#include <unistd.h>
#include <uuid.h>
#include <signal.h>

namespace
{
  const int DEFAULT_LOG_LEVEL{2};

  const int DEFAULT_PRIORITY_LEVEL{50};

  void usage(const std::string & sAppName,std::vector<std::string> additionalOptions)
  {
    std::cout<<"usage: "<<sAppName <<" [OPTIONS]... CONFIG_URL"<<std::endl;
    std::cout<<"usage: "<<sAppName <<" [OPTIONS]... --simulation"
             <<" CONFIG_URL"
             <<" ACTIVITY_SERVICE_SUBSCRIBE_ENDPOINT"
             <<" ACTIVITY_SERVICE_PUBLISH_ENDPOINT"
             <<" ACTIVITY_SERVICE_TAG"
             <<std::endl;
    std::cout<<std::endl;
    std::cout<<" CONFIG_URL                          URL of XML configuration file."<<std::endl;
    std::cout<<" ACTIVITY_SERVICE_SUBSCRIBE_ENDPOINT Activity Service subscribe endpoint."<<std::endl;
    std::cout<<" ACTIVITY_SERVICE_PUBLISH_ENPOINT    Activity Service publish endpoint."<<std::endl;
    std::cout<<" ACTIVITY_SERVICE_TAG                Activity Service tag."<<std::endl;
    std::cout<<std::endl;
    std::cout<<"options:"<<std::endl;
    std::cout<<"  -d, --daemonize                Run in the background."<<std::endl;
    std::cout<<"  -h, --help                     Print this message and exit."<<std::endl;
    std::cout<<"  -f, --logfile FILE             Log to a file instead of stdout."<<std::endl;
    std::cout<<"  -l, --loglevel [0,4]           Set initial log level."<<std::endl;
    std::cout<<"                                  default: "<<DEFAULT_LOG_LEVEL<<std::endl;
    std::cout<<"  --pidfile FILE                 Write application pid to file."<<std::endl;
    std::cout<<"  -p, --priority [0,99]          Set realtime priority level."<<std::endl;
    std::cout<<"                                 Only used with -r, --realtime."<<std::endl;
    std::cout<<"                                  default: "<<DEFAULT_PRIORITY_LEVEL<<std::endl;
    std::cout<<"  -r, --realtime                 Set realtime scheduling."<<std::endl;
    std::cout<<"  --simulation                   Enable simulation mode."<<std::endl;
    std::cout<<"  --uuidfile FILE                Write the application instance UUID to file."<<std::endl;
    std::cout<<"  -v, --version                  Print version and exit."<<std::endl;
    std::cout<<std::endl;

    if(!additionalOptions.empty())
      {
        std::cout<<"additional options:"<<std::endl;

        for(const auto & option : additionalOptions)
          {
            std::cerr<<option<<std::endl;
          }
        std::cout<<std::endl;
      }
  }

  std::mutex mutex{};

  EMANE::Application::ActivityCoordinator activityCoordinator{};

  void sighandler(int)
  {
    activityCoordinator.cancelWaitForStart();
    mutex.unlock();
  }
}

EMANE::Application::Main::Main(const std::string & sName):
  sName_{sName}
{
  uuid_generate(uuid_);

  EMANE::Application::initialize();
}

EMANE::Application::Main::~Main()
{
  EMANE::Application::shutdown();
}


const uuid_t & EMANE::Application::Main::getUUID() const
{
  return uuid_;
}

int EMANE::Application::Main::main(int argc, char * argv[])
{
  EMANE::Application::Logger logger{};

  try
    {
      std::vector<option> options =
        {
          {"help",     0, nullptr, 'h'},
          {"realtime", 0, nullptr, 'r'},
          {"loglevel", 1, nullptr, 'l'},
          {"logfile",  1, nullptr, 'f'},
          {"daemonize",0, nullptr, 'd'},
          {"version" , 0, nullptr, 'v'},
          {"pidfile" , 1, nullptr,  2},
          {"uuidfile", 1, nullptr,  3},
          {"priority", 1, nullptr,  'p'},
          {"simulation", 0, nullptr, 4},
        };

      std::string sOptString{"hrvdf:l:p:"};

      int iOption{};
      int iOptionIndex{};
      bool bDaemonize{};
      bool bRealtime{};
      int  iLogLevel{DEFAULT_LOG_LEVEL};
      int  iPriority{DEFAULT_PRIORITY_LEVEL};
      std::string sLogFile{};
      std::string sPIDFile{};
      std::string sUUIDFile{};
      bool bSimulation{};

      // add any specialized options
      std::vector<option> additionalOptions = doGetOptions();

      options.insert(options.end(),
                     additionalOptions.begin(),
                     additionalOptions.end());

      // add any specialized optsting items
      sOptString += doGetOptString();

      // close out the options
      options.push_back({0, 0,nullptr,0 });

      while((iOption = getopt_long(argc,argv,sOptString.c_str(),
                                   &options[0],
                                   &iOptionIndex)) != -1)
        {
          switch(iOption)
            {
            case 'h':
              usage(sName_,doGetOptionsUsage());
              return EXIT_SUCCESS;
              break;

            case 'r':
              bRealtime = true;
              if(optarg)
                {
                  std::cout<<optarg<<std::endl;
                }
              break;

            case 'd':
              bDaemonize = true;
              break;

            case 'v':
              std::cout<<VERSION<<std::endl;
              return EXIT_SUCCESS;

            case 'f':
              sLogFile = optarg;
              break;

            case 2:
              sPIDFile = optarg;
              break;

            case 3:
              sUUIDFile = optarg;
              break;

            case 'p':
              try
                {
                  iPriority = EMANE::Utils::ParameterConvert{optarg}.toINT32(0,99);
                }
              catch(...)
                {
                  std::cerr<<"invalid priority: "<<optarg<<std::endl;
                  return EXIT_FAILURE;
                }

              break;

            case 'l':
              try
                {
                  iLogLevel = EMANE::Utils::ParameterConvert{optarg}.toINT32(0,4);
                }
              catch(...)
                {
                  std::cerr<<"invalid log level: "<<optarg<<std::endl;
                  return EXIT_FAILURE;
                }

              break;

            case 4:
              bSimulation = true;
              break;

            case '?':
              if(optopt == 't')
                {
                  std::cerr<<"option -"<<static_cast<char>(optopt)<<" requires an argument."<<std::endl;
                }
              else if(isprint(optopt))
                {
                  std::cerr<<"unknown option -"<<static_cast<char>(optopt)<<"."<<std::endl;
                }
              else
                {
                  std::cerr<<"bad option"<<std::endl;
                }
              return EXIT_FAILURE;

            default:
              if(!doProcessOption(iOption,optarg))
                {
                  return EXIT_FAILURE;
                }
            }
        }

      INETAddr simulationCoordinatorEndpoint{};
      INETAddr activityServiceEndpoint{};
      std::string sActivityServiceTag{};

      std::string sConfigurationXML{};

      if(optind >= argc)
        {
          std::cerr<<"missing CONFIG_URL"<<std::endl;

          return EXIT_FAILURE;
        }
      else
        {
          sConfigurationXML = argv[optind++];

          if(bSimulation)
            {
              if(optind >= argc)
                {
                  std::cerr<<"missing SIMULATION_COORDINATOR_ENDPOINT"<<std::endl;
                  return EXIT_FAILURE;
                }
              else
                {
                  std::string sSimulationCoordinatorEndpoint{argv[optind++]};

                  try
                    {
                      simulationCoordinatorEndpoint =
                        EMANE::Utils::ParameterConvert{sSimulationCoordinatorEndpoint}.toINETAddr();
                    }
                  catch(...)
                    {
                      std::cerr<<"invalid SIMULATION_COORDINATOR_ENDPOINT: "
                               <<sSimulationCoordinatorEndpoint
                               <<std::endl;

                      return EXIT_FAILURE;
                    }
                }

              if(optind >= argc)
                {
                  std::cerr<<"missing ACTIVITY_SERVICE_ENDPOINT"<<std::endl;
                  return EXIT_FAILURE;
                }
              else
                {
                  std::string sActivityServiceEndpoint{argv[optind++]};

                  try
                    {
                      activityServiceEndpoint =
                        EMANE::Utils::ParameterConvert{sActivityServiceEndpoint}.toINETAddr();
                    }
                  catch(...)
                    {
                      std::cerr<<"invalid ACTIVITY_SERVICE_ENDPOINT: "
                               <<sActivityServiceEndpoint
                               <<std::endl;

                      return EXIT_FAILURE;
                    }
                }

              if(optind >= argc)
                {
                  std::cerr<<"missing ACTIVITY_SERVICE_TAG"<<std::endl;
                  return EXIT_FAILURE;
                }
              else
                {
                  sActivityServiceTag = argv[optind++];
                }
            }
        }

      if(bDaemonize)
        {
          if(sLogFile.empty() && iLogLevel != 0)
            {
              std::cerr<<"unable to daemonize log level must be 0 when logging to stdout"<<std::endl;;
              return EXIT_FAILURE;
            }

          if(daemon(1,0))
            {
              std::cerr<<"unable to daemonize"<<std::endl;
              return EXIT_FAILURE;
            }
        }

      if(!sLogFile.empty())
        {
          logger.redirectLogsToFile(sLogFile.c_str());
        }

      if(iLogLevel > 0)
        {
          logger.setLogLevel(static_cast<EMANE::LogLevel>(iLogLevel));

          logger.open();
        }
      else
        {
          logger.setLogLevel(EMANE::NOLOG_LEVEL);
        }

      //
      // Warning: Do not log BEFORE ActivityCordinator::open(Local|Remote)
      //
      if(bSimulation)
        {
          // using remote activity coordination (simulation)
          activityCoordinator.openRemote(uuid_,
                                         simulationCoordinatorEndpoint,
                                         activityServiceEndpoint,
                                         sActivityServiceTag);
        }
      else
        {
          // using local activity coordination (emulation)
          activityCoordinator.openLocal();
        }

      doConfigure(sConfigurationXML);

      std::stringstream ss;
      for(int i = 0; i < argc; ++i)
        {
          ss<<argv[i]<<" ";
        }

      char uuidBuf[37];
      uuid_unparse(uuid_,uuidBuf);

      logger.log(EMANE::INFO_LEVEL,"application: %s",ss.str().c_str());
      logger.log(EMANE::INFO_LEVEL,"application uuid: %s",uuidBuf);

      if(bRealtime)
        {
          struct sched_param schedParam{iPriority};

          if(sched_setscheduler(0,SCHED_RR,&schedParam))
            {
              if(!sLogFile.empty() || !iLogLevel)
                {
                  std::cerr<<"unable to set realtime scheduler"<<std::endl;
                }

              logger.log(EMANE::ABORT_LEVEL,"unable to set realtime scheduler");

              return EXIT_FAILURE;
            }
        }
      else
        {
          if(!sLogFile.empty() || iLogLevel < 2)
            {
              std::cerr<<"Please consider using the realtime scheduler to improve fidelity."<<std::endl;
            }

          logger.log(EMANE::ERROR_LEVEL,"Please consider using the realtime scheduler to improve fidelity.");
        }


      if(!sPIDFile.empty())
        {
          std::ofstream pidFile{sPIDFile.c_str(), std::ios::out};

          if(pidFile)
            {
              pidFile<<getpid()<<std::endl;
            }
          else
            {
              return EXIT_FAILURE;
            }
        }

      if(!sUUIDFile.empty())
        {
          std::ofstream uuidFile{sUUIDFile.c_str(), std::ios::out};

          if(uuidFile)
            {
              uuidFile<<uuidBuf<<std::endl;
            }
          else
            {
              return EXIT_FAILURE;
            }
        }


      struct sigaction action;

      memset(&action,0,sizeof(action));

      action.sa_handler = sighandler;
      sigaction(SIGINT,&action,nullptr);
      sigaction(SIGQUIT,&action,nullptr);

      activityCoordinator.waitForStart();

      logger.log(EMANE::DEBUG_LEVEL,
                 "wait for start fin");

      if(activityCoordinator.isStarted())
        {
          logger.log(EMANE::DEBUG_LEVEL,
                     "start detected");

          doStart();

          mutex.lock();

          // signal handler unlocks mutex
          mutex.lock();

          doStop();
        }

      doDestroy();
    }
  catch(const EMANE::Exception & exp)
    {
      logger.log(EMANE::ABORT_LEVEL,"%s: %s",exp.type(),exp.what());

      return EXIT_FAILURE;
    }
  catch(const std::exception & exp)
    {
      logger.log(EMANE::ABORT_LEVEL,"%s",exp.what());

      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
