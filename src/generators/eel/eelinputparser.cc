/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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
 *
 */

#include "eelinputparser.h"
#include "emane/generators/eel/formatexception.h"
#include "emane/utils/parameterconvert.h"

EMANE::Generators::EEL::InputParser::InputParser(){}

EMANE::Generators::EEL::InputParser::~InputParser(){}

bool EMANE::Generators::EEL::InputParser::parse(const std::string & sInput,
                                                float & fEventTime,
                                                std::string &sEventType,
                                                std::string &sModuleId,
                                                InputArguments & inputArguments)
{ 
  size_t pos = 0;
  int iArgumentCount = 0;
  std::string sArgument;

  // clear all input parameters
  fEventTime = 0;
  sEventType.clear();
  sModuleId.clear();
  inputArguments.clear();

  // strip leading white space
  pos = sInput.find_first_not_of(" \t");

  while(!(sArgument = getNextArgument(sInput, pos)).empty())
    {
      if(iArgumentCount == 0)
        {
          // first arguement is floating point time
          fEventTime  = 
            Utils::ParameterConvert(sArgument).toFloat();
        }
      else if(iArgumentCount == 1)
        {
          // second argument is module id
          sModuleId = sArgument;
        }
      else if(iArgumentCount == 2)
        {
          // third arguement id the event type
          sEventType = sArgument;
        }
      else
        {
          inputArguments.push_back(sArgument);
        }
      
      ++iArgumentCount;
    }
  
  return iArgumentCount >= 3;
}

std::string EMANE::Generators::EEL::InputParser::getNextArgument(const std::string & sInput,
                                                                 size_t & posStart)
{
  std::string sArgument;

  if(posStart !=  std::string::npos)
    {
      size_t posEnd = sInput.find_first_of(" \t\n#\"",posStart);

      if(posEnd != std::string::npos)
        {
          size_t len = 0;

          if(sInput.at(posEnd) == '\"')
            {
              if(sInput.at(posStart) == '\"')
                {
                  posEnd =  sInput.find_first_of("\"",posEnd + 1);
                
                  if(posEnd !=  std::string::npos)
                    {
                      ++posStart;
                      len =  posEnd - posStart;
                    }
                  else
                    {
                      // error: started an open string but did not finish it
                      std::stringstream ssDescription;
                      ssDescription<<"Unterminated string: "<<sInput<<std::ends;
                      throw FormatException(ssDescription.str());
                    }
                }
              else
                {
                  // error: found an open string not as the first char of an arguement
                  std::stringstream ssDescription;
                  ssDescription<<"Invalid start of string: "<<sInput<<std::ends;
                  throw FormatException(ssDescription.str());
                }
            }
          else
            {
              len = posEnd - posStart;
            }
          
          sArgument =  sInput.substr(posStart, len);

          posStart = sInput.find_first_not_of(" \t\n",posEnd);
        }
      else
        {
          sArgument =  sInput.substr(posStart);
          posStart  = std::string::npos;
        }
    }
  
  return sArgument;
}
