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

#include "configurationvalidator.h"

#include <map>

std::pair<std::string,bool>
EMANE::Models::IEEE80211ABG::configurationValidator(const ConfigurationUpdate & updates) noexcept
{
  std::map<std::string,std::vector<Any>>  parameters;

  std::transform(updates.begin(), 
                 updates.end(), 
                 std::inserter(parameters,parameters.end()),
                 [](const ConfigurationUpdate::value_type & p)
                 {
                   return std::make_pair(p.first,p.second);
                 });

  if(parameters["cwmin0"][0] >= parameters["cwmax0"][0])
    {
      return std::make_pair("cwmin0 must be less than cwmax0", false);
    }

  if(parameters["cwmin1"][0] >= parameters["cwmax1"][0])
    {
      return std::make_pair("cwmin1 must be less than cwmax1", false);
    }

  if(parameters["cwmin2"][0] >= parameters["cwmax2"][0])
    {
      return std::make_pair("cwmin2 must be less than cwmax2", false);
    }

  if(parameters["cwmin3"][0] >= parameters["cwmax3"][0])
    {
      return std::make_pair("cwmin3 must be less than cwmax3", false);
    }

  // 80211A
  if(parameters["mode"][0].asUINT8() == 1)
    {
      // unicast data rate index is only valid for 5 to 12 inclusive
      if(parameters["unicastrate"][0].asUINT8() < 5 || parameters["unicastrate"][0].asUINT8() > 12)
        {
          return std::make_pair("unicastrate range is [5-12] for mode 80211A", false);
        }

      // broadcast data rate index is only valid for 5 to 12 inclusive
      if(parameters["multicastrate"][0].asUINT8() < 5 || parameters["multicastrate"][0].asUINT8() > 12)
        {
          return std::make_pair("multicastrate range [5-12] for mode 80211A", false);
        }
    }

  // 80211B
  if(parameters["mode"][0].asUINT8() == 2 || parameters["mode"][0].asUINT8() == 0)
    {
      // unicast data rate index is only valid for 1 to 4 inclusive
      if(parameters["unicastrate"][0].asUINT8() < 1 || parameters["unicastrate"][0].asUINT8() > 4)
        {
          return std::make_pair("unicastrate range [1-4] for mode 80211B", false);
        }

      // broadcast data rate index is only valid for 1 to 4 inclusive
      if(parameters["multicastrate"][0].asUINT8() < 1 || parameters["multicastrate"][0].asUINT8() > 4)
        {
          return std::make_pair("multicastrate range [1-4] for mode 80211B", false);
        }
    }

  // 80211BG
  if(parameters["mode"][0].asUINT8() == 3)
    {
      // unicast data rate index is only valid for 1 to 12 inclusive
      if(parameters["unicastrate"][0].asUINT8() < 1 || parameters["unicastrate"][0].asUINT8() > 12)
        {
          return std::make_pair("unicastrate range [1-12] for mode 80211BG", false);
        }

      // broadcast data rate index is only valid for 1 to 12 inclusive
      if(parameters["multicastrate"][0].asUINT8() < 1 || parameters["multicastrate"][0].asUINT8() > 12)
        {
          return std::make_pair("multicastrate range [1-12] for mode 80211BG", false);
        }
    }


  return std::make_pair("", true);
}
