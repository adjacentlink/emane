/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2009-2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "configurationparser.h"


namespace 
{
  /* DTD validation options */
  const int DEFAULT_PARSER_OPTIONS = XML_PARSE_NOENT |    //sub entities
                                     XML_PARSE_DTDLOAD |  //load ext subset
                                     XML_PARSE_DTDATTR |  //default attr vals
                                     XML_PARSE_DTDVALID;  //validate w/dtd
}


EMANE::ConfigurationParser::ConfigurationParser()
{
  xmlInitParser();

  pContext_ = xmlNewParserCtxt();

  if(!pContext_)
    {
      throw ParseException{
        "XML Parser initialized incorrectly (Context is NULL)"};
    }
}


EMANE::ConfigurationParser::~ConfigurationParser()
{
  xmlFreeParserCtxt(pContext_);

  xmlCleanupParser();
}


xmlDocPtr 
EMANE::ConfigurationParser::parse(const std::string &sURI)
{
  xmlInitParserCtxt(pContext_);

  /* Allocate document */
  xmlDocPtr pDoc = xmlCtxtReadFile(pContext_,
                                   sURI.c_str(),
                                   0,
                                   DEFAULT_PARSER_OPTIONS);

  if (!pDoc)
    {
      throw makeException<ParseException>(
                         "Failed to parse document in %s.\n\n"
                         "Possible reason(s):\n"
                         " * Document '%s' does not exist.\n", 
                         sURI.c_str(), 
                         sURI.c_str());
    }

  /* Parse was successful, check if valid */
  if (pContext_->valid == 0)
    {
      /* Data failed to validate */

      xmlDtdPtr pDTD = xmlGetIntSubset(pDoc);
      
      ValidateException validationException{
            makeException<ValidateException>(
                    "Failed to parse document in %s.\n\n"
                    "Possible reason(s):\n"
                    " * %s is missing'\n"
                    " * %s inaccessible (network unreachable/http down).\n"
                    " * %s is incorrect (run xmllint to validate).\n",
                    sURI.c_str(), 
                    (pDTD ? (const char *) pDTD->SystemID : "DTD"),
                    (pDTD ? (const char *) pDTD->SystemID : "DTD"),
                    (pDTD ? (const char *) pDTD->SystemID : "DTD"))};

      xmlFreeDoc(pDoc);

      throw validationException;
    }
  
  /* Everything checks out, return the document */
  return pDoc;
}
