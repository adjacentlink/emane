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

#ifndef EMANESPECTRUMWINDOWUTILS_HEADER_
#define EMANESPECTRUMWINDOWUTILS_HEADER_

#include "emane/spectrumserviceprovider.h"
#include "emane/spectrumserviceexception.h"

#include <vector>

namespace EMANE
{
  namespace Utils
  {
    /**
     * Gets the noise floor for a given time range and spectrum window using
     * the maximum bin power within the range.
     *
     * @param window Spectrum window to analyze
     * @param dRxPowerdBm Signal power level in dBm
     * @param startTime Start time of the range
     * @param endTime End time of the range. The default value indicates the entire window.
     *
     * @return A pair indicating the noise floor in dBm and a boolean flag set @a true if
     * the in-band signal was contained in the bin.
     *
     * @throw SpectrumServiceException when an error is detected with one or more input
     * parameters.
     */
    std::pair<double,bool> maxBinNoiseFloorRange(const SpectrumWindow & window,
                                                 double dRxPowerdBm,
                                                 const TimePoint & startTime,
                                                 const TimePoint & endTime = TimePoint::min());

    /**
     * Gets the noise floor for a given start time and spectrum window using
     * the maximum bin power.
     *
     * @param window Spectrum window to analyze
     * @param dRxPowerdBm Signal power level in dBm
     * @param startTime Start time. The default value indicates start of the window.
     *
     * @return A pair indicating the noise floor in dBm and a boolean flag set @a true if
     * the in-band signal was contained in the bin.
     *
     * @throw SpectrumServiceException when an error is detected with one or more input
     * parameters.
     */
    std::pair<double,bool> maxBinNoiseFloor(const SpectrumWindow & window,
                                            double dRxPowerdBm,
                                            const TimePoint & startTime = TimePoint::min());
    
    /**
     * Gets the noise floor for a given start and end bin index and spectrum window bin
     * data using the maximum bin power.
     *
     * @param noiseData Spectrum window bin data to analyze
     * @param dRxSensitivityMilliWatt Receiver sensitivity in mW
     * @param dRxPowerdBm Signal power level in dBm
     * @param startBin Start bin index
     * @param startBin End bin index
     *
     * @return A pair indicating the noise floor in dBm and a boolean flag set @a true if
     * the in-band signal was contained in the bin.
     *
     * @throw SpectrumServiceException when an error is detected with one or more input
     * parameters.
     */
    std::pair<double,bool> maxBinNoiseFloor(const std::vector<double> & noiseData,
                                            double dRxSensitivityMilliWatt,
                                            double dRxPowerdBm,
                                            bool bSignalInNoise,
                                            std::size_t startBin,
                                            std::size_t endBin);

    
    /**
     * Gets the absolute bin of a specified time point from the epoch.
     * 
     * @param timePoint Desired time
     * @param binSize  FrameworkPHY bin size
     * @param bAdjust Flag that when @a true adjusts to the previous bin if
     * the time falls on a bin boundary
     *
     * @return absolute bin
     *
     * @note The adjustment flag is useful when determining the end bin in a given
     * range. If the end time falls on a boundary than you are really interested in
     * the previous bin.
     */
    Microseconds::rep timepointToAbsoluteBin(const TimePoint & timePoint,
                                             const Microseconds & binSize,
                                             bool bAdjust);

    using SpectrumCompressedRepresentation = std::vector<std::pair<std::size_t,float>>;

    /** 
     * Compresses spectrum window data into a smaller format.
     *
     * @param window A vector of spectrum window data
     *
     * @return compressed representation
     * 
     * Compression format consists of wheel-index wheel-value pairs where:
     *  (1) An implicit initial pair index=0, value=0 exists unless index 0
     *      is otherwise specified
     *  (2) An index value is valid until a new index value is specified
     *  (3) The last index value pair is valid through the end of the wheel
     *
     * Examples if the wheel contained 100 values and:
     *   (1) All values were 0, the entire CompressedRepresentation would be empty: {}
     *   (2) First 50 values were 1 and the next 50 values were 2: {{0,1},{50,2}}
     *   (3) First 10 values were 1 and the last 10 values were 1: {{0,1},{10,0},{90,1}}
     *
     * @note This is currently used in the standalone test cases
     */
    SpectrumCompressedRepresentation spectrumCompress(const std::vector<double> & window);
  }
}

#endif //EMANESPECTRUMWINDOWUTILS_HEADER_
