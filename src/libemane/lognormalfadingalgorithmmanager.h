/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that:
 *
 * (1) source code distributions retain this paragraph in its entirety,
 *
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided
 *     with the distribution.
 *
 *      "This product includes software written and developed
 *       by Code 5520 of the Naval Research Laboratory (NRL)."
 *
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef EMANELOGNORMALFADINGALGORITHMMANAGER_HEADER_
#define EMANELOGNORMALFADINGALGORITHMMANAGER_HEADER_

#include "fadingalgorithmmanager.h"
#include "lognormalfadingalgorithm.h"

namespace EMANE
{
  class LognormalFadingAlgorithmManager : public FadingAlgorithmManager
  {
  public:
    LognormalFadingAlgorithmManager(NEMId id,
                                   PlatformServiceProvider * pPlatformService,
                                   const std::string & sPrefix);

    void initialize(Registrar & registrar) override;

    void configure(const ConfigurationUpdate & update) override;

    void modify(const ConfigurationUpdate & update) override;

    std::unique_ptr<FadingAlgorithm> createFadingAlgorithm() override;

    const void * getParameters() const override;

  private:
    LognormalFadingAlgorithm::Parameters parameters_;

    void configure_i(const ConfigurationUpdate & update);
  };
};

#endif // EMANELOGNORMALFADINGALGORITHMMANAGER_HEADER_
