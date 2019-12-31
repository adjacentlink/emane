/*
 * Copyright (c) 2013-2014,2019-2020 - Adjacent Link LLC, Bridgewater,
 * New Jersey
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

#ifndef EMANESPECTRUMSERVICEPROVIDER_HEADER_
#define EMANESPECTRUMSERVICEPROVIDER_HEADER_

#include "emane/types.h"

#include <set>
#include <vector>
#include <tuple>

namespace EMANE
{
  using FrequencySet = std::set<std::uint64_t>;

  /**
   * Spectrum window snapshot
   *
   * @param std::vector<double> A vector of binned signal energy in mW
   * @param TimePoint Time corresponding to the first bin
   * @param Microseconds The bin duration
   * @param double The receiver sensitivity in mW
   * @param bool Flag indicating whether the in-band signal
   * is contained in the binned power values
   *
   * @note If you remove the in-band signal from a bin and the remaining bin
   * energy is 0 mW you need to use the receiver sensitivity for the bin value.
   * You cannot have less than receiver sensitivity in any bin.
   *
   * @see Utils::maxBinNoiseFloor
   */
  using SpectrumWindow = std::tuple<std::vector<double>,
                                    TimePoint,
                                    Microseconds,
                                    double,
                                    bool>;

  /**
   * Spectrum filter window snapshot
   *
   * @param std::vector<double> A vector of binned signal energy in mW
   * @param TimePoint Time corresponding to the first bin
   * @param Microseconds The bin duration
   * @param double The receiver sensitivity in mW
   * @param bool Flag indicating whether the in-band signal
   * is contained in the binned power values
   * @param size_t Number of subband binned signal energy per time bin
   */
  using SpectrumFilterWindow = std::tuple<std::vector<double>,
                                          TimePoint,
                                          Microseconds,
                                          double,
                                          bool,
                                          size_t>;

  /**
   * @class SpectrumServiceProvider
   *
   * @brief The SpectrumServiceProvider interface provides an API to request
   * noise window information
   *
   * @note Spectrum services are only available when using the FrameworkPHY,
   * the emualtor's built in physical layer.
   */
  class SpectrumServiceProvider
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~SpectrumServiceProvider(){};


    /**
     * Gets the set of monitored frequencies
     *
     * @return frequency set
     */
    virtual FrequencySet getFrequencies() const = 0;

    /**
     * Gets the receiver sensitivity in dBm
     *
     * @return receiver sensitivity
     */
    virtual double getReceiverSensitivitydBm() const = 0;

    /**
     * Gets a spectrum window
     *
     * @param u64FrequencyHz Frequency to query for its spectrum window
     * @param duration The amount of time in microseconds to request. If not specified
     * the default will indicate maximum duration configured for the FrameworkPHY (default
     * only recommended for DSA type functionality).
     * @param startTime The start time of the spectrum window request. If not
     * specified the default value will indicate now -  maximum duration configured for the
     * FrameworkPHY (default only recommended for DSA type functionality).
     *
     * @pre The requested frequency must me a monitored frequency
     * @pre The duration cannot be more than the maximum duration configured for
     * the FrameworkPHY
     * @pre The start time plus the duration cannot be greater than @c now
     * @pre The start time cannot be less than @c now minus the maximum duration
     * configured for the FrameworkPHY
     *
     * @return A spectrum window
     *
     * @throw SpectrumServiceException when the request parameters violate preconditions.
     *
     * @note Request duration should almost always be the span contained in the
     * Controls::ReceivePropertiesControlMessage.
     *
     * @note Request startTime should be the start of reception (SoR) which is the start of
     * transmission (SoT) + propagation delay + offset of the first frequency segment. SoT
     * and propagation delay are contained in the Controls::ReceivePropertiesControlMessage.
     * Frequency segment offset is contained in the Controls::FrequencyControlMessage.
     *
     * @note Can be passed to Utils::maxBinNoiseFloorRange or Utils::maxBinNoiseFloor.
     */

    virtual SpectrumWindow request(std::uint64_t u64FrequencyHz,
                                   const Microseconds & duration = Microseconds::zero(),
                                   const TimePoint & startTime = TimePoint::min()) const = 0;


    /**
     * Gets a filter spectrum window
     *
     * @param filterIndex Filter index to query for its spectrum window
     * @param duration The amount of time in microseconds to request. If not specified
     * the default will indicate maximum duration configured for the FrameworkPHY (default
     * only recommended for DSA type functionality).
     * @param startTime The start time of the spectrum window request. If not
     * specified the default value will indicate now -  maximum duration configured for the
     * FrameworkPHY.
     *
     * @pre The requested filter id must exist.
     * @pre The duration cannot be more than the maximum duration configured for
     * the FrameworkPHY
     * @pre The start time plus the duration cannot be greater than @c now
     * @pre The start time cannot be less than @c now minus the maximum duration
     * configured for the FrameworkPHY
     *
     * @return A spectrum fitler window
     *
     * @throw SpectrumServiceException when the request parameters violate preconditions.
     *
     * @note Request duration should almost always be the span contained in the
     * Controls::ReceivePropertiesControlMessage.
     *
     * @note Request startTime should be the start of reception (SoR) which is the start of
     * transmission (SoT) + propagation delay + offset of the first frequency segment. SoT
     * and propagation delay are contained in the Controls::ReceivePropertiesControlMessage.
     * Frequency segment offset is contained in the Controls::FrequencyControlMessage.
     */

    virtual SpectrumFilterWindow requestFilter(FilterIndex filterIndex,
                                               const Microseconds & duration = Microseconds::zero(),
                                               const TimePoint & startTime = TimePoint::min()) const = 0;

  protected:
    SpectrumServiceProvider() = default;
  };
}

#endif //EMANESPECTRUMSERVICEPROVIDER_HEADER_

/**
 * @page SpectrumService Spectrum Service
 *
 * The @ref EMANE::SpectrumServiceProvider "Spectrum Service" provides NEM component layers with
 * access to the spectrum monitoring capabilities of the @ref EMANE::FrameworkPHY "emulator's physical layer".
 *
 * Each emulator physical layer instance can be configured to monitor a frequency of interest set. A
 * @ref EMANE::NoiseRecorder "noise recorder" is instantiated for each frequency of interest. The noise
 * recorder keeps track of signal energy and can be queried at a specified time for a @ref EMANE::SpectrumWindow
 * "spectrum window". A spectrum window contains binned signal energies across a requested duration along with
 * the information necessary to interpret the data.
 *
 * Each over-the-air  message is checked for frequency overlap using the transmitter and receiver
 * bandwidth. If there is an overlap, a proportional amount of signal energy is applied.
 *
 * There are three types of noise recording modes:
 * - none
 * - out-of-band
 * - all
 *
 * Noise recording mode @a none does not record any signal energy and will always return 0 mW bins
 * whenever queried.
 *
 * Noise recording mode @a out-of-band will only record signal energy for out-of-band messages. An
 * out-of-band message is one that does not match the physical layer subid or a message that matches the
 * subid but contains one or more frequency segments with a frequency that is not in the configured
 * frequency of interest set.
 *
 * Noise recording mode @a all will record all signal energy including in-band signals. This will require
 * subtracting the in-band signal energy in order to determine the noise.
 *
 * @section NoiseRecording Noise Recording
 *
 * Spectrum energy is recorded on a @ref EMANE::Wheel "wheel of bins". Bin width in microseconds can be
 * configured as part of the @ref EMANE::FrameworkPHY "emulator's physical layer" configuration. When an
 * over-the-air message is received, the receive power is calculated based on the configured propagation
 * model and antenna gain profiles. If the receive power is above the receiver sensitivity, it is applied
 * to one or more wheel bins based on the signal duration.
 *
 * Signal energy is applied at the start-of-reception time which is the: start-of-transmission +
 * propagation + first frequency segment offset. The emulator's physical layer is configured with the
 * maximum allowable message duration, propagation time and frequency segment offset. By default, messages
 * sent containing values above the configured maximums will be dropped.
 *
 * @section SpectrumWindow Interpreting a Spectrum Window
 *
 * An NEM component layer can query for a spectrum window by specifying a frequency, start time and duration.
 * The frequency must be in the configured frequency of interest set and the start time cannot be earlier
 * than @a now - max duration or later than @a now.
 *
 * The EMANE::SpectrumServiceProvider::request method is used to request a spectrum window. The following
 * example is taken from the @ref  models/mac/rfpipe/maclayer.cc "RF Pipe MAC layer" implementation and
 * shows a spectrum window request and noise floor determination using the EMANE::Utils::maxBinNoiseFloor
 * utility function.
 *
 * @snippet models/mac/rfpipe/maclayer.cc spectrumservice-request-snibbet
 *
 * You must wait for the end-of-reception time prior to requesting a spectrum window in order to account for
 * all the signal energy that occurs during the message duration, where end-of-reception =
 * start-of-reception + message duration.
 *
 * The emulator's physical layer will pass along an EMANE::Controls::ReceivePropertiesControlMessage and an
 * EMANE::Controls::FrequencyControlMessage with each upstream packet.
 *
 * The EMANE::Controls::ReceivePropertiesControlMessage contains the message start-of-transmission time, the
 * message propagation delay, the message span and the receiver sensitivity. The message span is the total
 * time between the start of the signal's earliest frequency segment and the end of the signal's latest
 * frequency segment. The span is supplied to make spectrum window querying easier.
 *
 * It is best practice to always use the span when requesting a spectrum window even if you are only interested
 * in one or more smaller subsets of the window.  This will allow you to analyze a single window for one or more
 * subsets without making additional requests to the spectrum service.
 *
 * For messages with one or more frequencies, a spectrum window should be requested for each of the frequencies.
 *
 * The EMANE::SpectrumWindow contains:
 * - Vector of binned signal energies in mW
 * - The time of the first bin
 * - The bin duration in microseconds
 * - The receiver sensativity in dBm
 * - A flag indicating whether the in-band signal energy is contained in the bins
 *
 * The EMANE::Utils::maxBinNoiseFloor utility function will use the maximum binned signal energy in a spectrum
 * window (or subset of a spectrum window) as the noise floor. If the in-band signal is contained in the binned value
 * it will be removed and if no additional energy is present in the bin, the receiver sensitivity will be used as the
 * noise floor.
 */
