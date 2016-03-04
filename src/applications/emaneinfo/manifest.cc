/*
 * Copyright (c) 2013,2015 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include <libxml/parser.h>

#include "emane/application/configurationcontroller.h"
#include "emane/application/statisticcontroller.h"

namespace EMANE
{
  namespace Application
  {
std::string manifest(BuildId buildId, const std::string & sName)
{
  xmlDocPtr pDoc{xmlNewDoc(BAD_CAST "1.0")};

  xmlNodePtr pManifest{xmlNewNode(NULL,BAD_CAST "manifest")};

  xmlNodePtr pPlugin{xmlNewChild(pManifest,NULL,BAD_CAST "plugin",NULL)};

  xmlNewProp(pPlugin, BAD_CAST "name", BAD_CAST sName.c_str());

  auto manifest =
    ConfigurationController::getConfigurationManifest(buildId);

  xmlNodePtr pConfiguration{xmlNewChild(pPlugin,NULL,BAD_CAST "configuration",NULL)};

  for(const auto & entry : manifest)
    {
      xmlNodePtr pParam{xmlNewChild(pConfiguration,NULL,BAD_CAST "parameter",NULL)};

      xmlNewProp(pParam, BAD_CAST "name", BAD_CAST entry.getName().c_str());
      xmlNewProp(pParam, BAD_CAST "default", BAD_CAST (entry.hasDefault() ? "yes" : "no"));
      xmlNewProp(pParam, BAD_CAST "required", BAD_CAST (entry.isRequired() ? "yes" : "no"));
      xmlNewProp(pParam,  BAD_CAST "modifiable", BAD_CAST (entry.isModifiable() ? "yes" : "no"));


      std::string sType;
      std::string sMinValue;
      std::string sMaxValue;
      bool bNumeric{true};

      std::vector<std::string> values;

      const auto & anyValues = entry.getValues();

      switch(entry.getType())
        {
        case Any::Type::TYPE_INT64:
          sType="int64";
          sMinValue = std::to_string(entry.getMinValue().asINT64());
          sMaxValue = std::to_string(entry.getMaxValue().asINT64());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asINT64());
                         });

          break;

        case Any::Type::TYPE_UINT64:
          sType="uint64";
          sMinValue = std::to_string(entry.getMinValue().asUINT64());
          sMaxValue = std::to_string(entry.getMaxValue().asUINT64());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asUINT64());
                         });
          break;

        case Any::Type::TYPE_INT32:
          sType="int32";
          sMinValue = std::to_string(entry.getMinValue().asINT32());
          sMaxValue = std::to_string(entry.getMaxValue().asINT32());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asINT32());
                         });
          break;

        case Any::Type::TYPE_UINT32:
          sType="uint32";
          sMinValue = std::to_string(entry.getMinValue().asUINT32());
          sMaxValue = std::to_string(entry.getMaxValue().asUINT32());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asUINT32());
                         });

          break;

        case Any::Type::TYPE_INT16:
          sType="int16";
          sMinValue = std::to_string(entry.getMinValue().asINT16());
          sMaxValue = std::to_string(entry.getMaxValue().asINT16());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asINT16());
                         });

          break;

        case Any::Type::TYPE_UINT16:
          sType="uint16";
          sMinValue = std::to_string(entry.getMinValue().asUINT16());
          sMaxValue = std::to_string(entry.getMaxValue().asUINT16());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asUINT16());
                         });

          break;

        case Any::Type::TYPE_INT8:
          sType="int8";
          sMinValue = std::to_string(entry.getMinValue().asINT8());
          sMaxValue = std::to_string(entry.getMaxValue().asINT8());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asINT8());
                         });
          break;

        case Any::Type::TYPE_UINT8:
          sType="uint8";
          sMinValue = std::to_string(entry.getMinValue().asUINT8());
          sMaxValue = std::to_string(entry.getMaxValue().asUINT8());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asUINT8());
                         });
          break;

        case Any::Type::TYPE_FLOAT:
          sType="float";
          sMinValue = std::to_string(entry.getMinValue().asFloat());
          sMaxValue = std::to_string(entry.getMaxValue().asFloat());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asFloat());
                         });
          break;

        case Any::Type::TYPE_DOUBLE:
          sType="double";
          sMinValue = std::to_string(entry.getMinValue().asDouble());
          sMaxValue = std::to_string(entry.getMaxValue().asDouble());

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return std::to_string(any.asDouble());
                         });
          break;

        case Any::Type::TYPE_INET_ADDR:
          bNumeric = false;
          sType="inetaddr";

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return any.asINETAddr().str();
                         });

          break;

        case Any::Type::TYPE_BOOL:
          sType="bool";
          sMinValue = "false";
          sMaxValue = "true";

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return any.asBool() ? "true" : "false";
                         });

          break;

        case Any::Type::TYPE_STRING:
          bNumeric = false;
          sType="string";

          std::transform(anyValues.begin(),
                         anyValues.end(),
                         std::back_inserter(values),[](const Any & any)->std::string
                         {
                           return any.asString();
                         });
          break;
        }

      xmlNodePtr pNumericOrNonNumeric{};

      if(bNumeric)
        {
          pNumericOrNonNumeric = xmlNewChild(pParam,NULL,BAD_CAST "numeric",NULL);
          xmlNewProp(pNumericOrNonNumeric, BAD_CAST "minValue", BAD_CAST sMinValue.c_str());
          xmlNewProp(pNumericOrNonNumeric,BAD_CAST "maxValue", BAD_CAST sMaxValue.c_str());
        }
      else
        {
          pNumericOrNonNumeric = xmlNewChild(pParam,NULL,BAD_CAST "nonnumeric",NULL);
        }

      const auto & sPattern = entry.getRegexPattern();

      xmlNewProp(pNumericOrNonNumeric, BAD_CAST "type", BAD_CAST sType.c_str());

      xmlNodePtr pValues = xmlNewChild(pNumericOrNonNumeric, NULL, BAD_CAST "values", NULL);
      xmlNewProp(pValues,BAD_CAST "minOccurs", BAD_CAST std::to_string(entry.getMinOccurs()).c_str());
      xmlNewProp(pValues,BAD_CAST "maxOccurs", BAD_CAST std::to_string(entry.getMaxOccurs()).c_str());


      std::for_each(values.begin(),
                    values.end(),
                    [pValues](const std::string & s)
                    {
                      xmlNewChild(pValues, NULL, BAD_CAST "value", BAD_CAST s.c_str());
                    });


      if(!sPattern.empty())
        {
          xmlNodePtr pRegex = xmlNewChild(pNumericOrNonNumeric, NULL, BAD_CAST "regex", NULL);
          xmlNodePtr pCDATA = xmlNewCDataBlock(pDoc, BAD_CAST sPattern.c_str(), sPattern.size());
          xmlAddChild(pRegex,pCDATA);
        }

      if(!entry.getUsage().empty())
        {
          xmlNewChild(pNumericOrNonNumeric, NULL, BAD_CAST "description", BAD_CAST entry.getUsage().c_str());
        }
    }


  xmlNodePtr pStatistic{xmlNewChild(pPlugin,NULL,BAD_CAST "statistics",NULL)};

  for(const auto & entry : StatisticController::getStatisticManifest(buildId))
    {
      xmlNodePtr pStat{xmlNewChild(pStatistic,NULL,BAD_CAST "element",NULL)};

      xmlNewProp(pStat, BAD_CAST "name", BAD_CAST entry.getName().c_str());

      xmlNewProp(pStat, BAD_CAST "type", BAD_CAST anyTypeAsString(entry.getType()).c_str());

      xmlNewProp(pStat, BAD_CAST "clearable", BAD_CAST (entry.isClearable() ? "yes" : "no"));

      if(!entry.getDescription().empty())
        {
          xmlNewChild(pStat, NULL, BAD_CAST "description", BAD_CAST entry.getDescription().c_str());
        }
    }

   xmlNodePtr pStatisticTables{xmlNewChild(pPlugin,NULL,BAD_CAST "statistictables",NULL)};

   for(const auto & entry : StatisticController::getTableManifest(buildId))
     {
       xmlNodePtr pTable{xmlNewChild(pStatisticTables,NULL,BAD_CAST "table",NULL)};

       xmlNewProp(pTable, BAD_CAST "name", BAD_CAST entry.getName().c_str());

       xmlNewProp(pTable, BAD_CAST "clearable", BAD_CAST (entry.isClearable() ? "yes" : "no"));

       if(!entry.getDescription().empty())
        {
          xmlNewChild(pTable, NULL, BAD_CAST "description", BAD_CAST entry.getDescription().c_str());
        }
     }

  xmlDocSetRootElement(pDoc,pManifest);

  xmlChar * xmlbuff;

  int buffersize;

  xmlDocDumpFormatMemory(pDoc, &xmlbuff, &buffersize, 1);

  std::string sManifest(reinterpret_cast<char *>(xmlbuff),buffersize);

  xmlFree(xmlbuff);

  xmlFreeDoc(pDoc);

  return sManifest;
}
  }
}
