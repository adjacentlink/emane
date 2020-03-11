/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANECONFIGURATIONUPDATE_HEADER_
#define EMANECONFIGURATIONUPDATE_HEADER_

#include "emane/configurationinfo.h"

#include <vector>
#include <string>
#include <functional>

namespace EMANE
{
  /**
   * Configuration item name and values as strings
   *
   * @param std::string Configuration item name
   * @param std::vector<std::string> Vector of configuration values
   */
  using ConfigurationNameStringValues =
    std::pair<std::string,std::vector<std::string>>;

  /**
   * Configuration update request consisting of item names and
   * their values as strings
   *
   * @param std::vector<ConfigurationNameStringValues> A Vector of
   * configuration items and their values
   */
  using ConfigurationUpdateRequest =
    std::vector<ConfigurationNameStringValues>;

  /** [configurationregistrar-configurationnameanyvalues-snippet] */

  /**
   *  Configuration item name and values as EMANE::Any instances
   *
   * @param std::string Configuration item name
   * @param std::vector<EMANE::Any> A Vector of configuration values
   */
  using ConfigurationNameAnyValues =
    std::pair<std::string,std::vector<EMANE::Any>>;

  /**
   * Configuration update consisting of item names and
   * their values as EMANE::Any instances
   *
   * @param std::vector<ConfigurationNameAnyValues> A Vector of
   * configuration items and their values
   */
  using ConfigurationUpdate =
    std::vector<ConfigurationNameAnyValues>;

  /** [configurationregistrar-configurationnameanyvalues-snippet] */

  /**
   * Configuration item manifest
   *
   * @param std::vector<ConfigurationInfo> A Vector of
   * EMANE::ConfigurationInfo entries
   */
  using ConfigurationManifest =
    std::vector<ConfigurationInfo>;

  /**
   * Callback used to validate configuration item relationships
   *
   * @param update The complete configuration including any newly requested changes
   *
   * @return A pair where @a first is an error description @a second is a boolean
   * which indicates an error when set @a false.
   */
  using ConfigurationValidator =
    std::function<std::pair<std::string,bool>(const ConfigurationUpdate & update)>;
};

#endif //EMANECONFIGURATIONUPDATE_HEADER_
