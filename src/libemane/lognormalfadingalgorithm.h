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

#ifndef EMANELOGNORMALFADINGALGORITHM_HEADER_
#define EMANELOGNORMALFADINGALGORITHM_HEADER_

#include "fadingalgorithm.h"
#include "emane/utils/conversionutils.h"
#include <random>

namespace EMANE
{
  class LognormalFadingAlgorithm: public FadingAlgorithm
  {
  public:
    LognormalFadingAlgorithm(NEMId id,
                            PlatformServiceProvider * pPlatformService);

    ~LognormalFadingAlgorithm();

    struct Parameters
    {
      double dmu_{};
      double dsigma_{};
      double dlthresh_{};
      double maxpathloss_{};
      double duthresh_{};
      double minpathloss_{};
      double lmean_{};
      double lstddev_{};
      unsigned int counter_{};
    };

    double operator()(double dPowerdBm, double dDistanceMeters, const void * pParams) override
    {
      auto pLognormalFadingParameters = reinterpret_cast<const Parameters *>(pParams);

      if(pLognormalFadingParameters->counter_ != param_counter_)
        {
          reset_ = true;
          param_counter_ = pLognormalFadingParameters->counter_;
	}

      TimePoint now=Clock::now();
      if(reset_ || now>=nexttime_)
        {
          // start new fading period
          // ignoring any difference in time from end of last fading period to now
          nexttime_=now+std::chrono::duration_cast< std::chrono::milliseconds >((DoubleSeconds)ldistribution_(generator_, NormalDistribution::param_type{pLognormalFadingParameters->lmean_, pLognormalFadingParameters->lstddev_}));
          double depthNorm=ddistribution_(generator_, LognormalDistribution::param_type{pLognormalFadingParameters->dmu_, pLognormalFadingParameters->dsigma_});
          // convert normalized depth into dBm depth
          if(depthNorm <= pLognormalFadingParameters->dlthresh_)
            {
              depthdBm_ = pLognormalFadingParameters->maxpathloss_;
            }
          else if(depthNorm >= pLognormalFadingParameters->duthresh_)
            {
              depthdBm_ = pLognormalFadingParameters->minpathloss_;
            }
          else
            {
              // linear approximation
              depthdBm_ = ((depthNorm - pLognormalFadingParameters->dlthresh_) / (pLognormalFadingParameters->duthresh_ - pLognormalFadingParameters->dlthresh_)) * (pLognormalFadingParameters->minpathloss_ - pLognormalFadingParameters->maxpathloss_) + pLognormalFadingParameters->maxpathloss_;
            }
	  reset_ = false;
	}
      return Utils::DB_TO_MILLIWATT(dPowerdBm-depthdBm_);
    }

  private:
    unsigned int param_counter_ = 0;
    bool reset_ = true;
    double depthdBm_;
    TimePoint nexttime_;
    std::mt19937 generator_;
    using  LognormalDistribution = std::lognormal_distribution<>;
    LognormalDistribution ddistribution_;
    using  NormalDistribution = std::normal_distribution<>;
    NormalDistribution ldistribution_;
  };
}

#endif // EMANELOGNORMALFADINGALGORITHM_HEADER_
