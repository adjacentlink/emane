/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef CONFIGURATIONPARSER_HEADER_
#define CONFIGURATIONPARSER_HEADER_

#include "emaneparseexception.h"

#include <string>

#include <libxml/parser.h>
#include <libxml/tree.h>


namespace EMANE
{
  /**
   * @class ConfigurationParser
   *
   * @brief Wrapper around the libxml2 XML parsing capabilities for EMANE
   */
  class ConfigurationParser
  {
  public:
    /**
     * Default constructor
     */
    ConfigurationParser();

    /**
     * Destructor
     */
    ~ConfigurationParser();

    /**
     * Parses the specified URI
     *
     * @param sURI URI to be parsed
     *
     * @return pDoc Pointer to the parsed document, NULL on failure.
     *              It is up to the caller to free the document.
     *
     * @exception ParseException ValidateException
     */
    xmlDocPtr parse(const std::string &sURI);
    
  private:
    /**
     * Parser context for libxml2
     */
    xmlParserCtxtPtr pContext_;

  };
}

#endif /* CONFIGURATIONPARSER_HEADER_ */
