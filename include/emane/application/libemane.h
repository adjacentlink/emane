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

#ifndef EMANEAPPLICATIONLIBEMANE_HEADER_
#define EMANEAPPLICATIONLIBEMANE_HEADER_

namespace EMANE
{
  namespace Application
  {
    /**
     * Initialize libemane
     *
     * @note Must be called once prior to using any library APIs
     */
    void initialize();
    
    /**
     * Shutdown libemane
     *
     * @note Must be called last after library usage is complete
     */
    void shutdown();
  }
}

#endif // EMANEAPPLICATIONLIBEMANE

/**
 * @mainpage EMANE
 *
 * %EMANE (Extendable Mobile Ad-hoc Network Emulator) is an open source distributed emulation
 * framework which provides wireless network experimenters with a highly flexible modular
 * environment for use during the design, development and testing of simple and complex network
 * architectures. %EMANE uses a physical layer model to account for signal propagation, antenna
 * profile effects and interference sources in order to provide a realistic environment for
 * wireless experimentation. Individual radio model plugins are used to emulate the lowest layers
 * of a waveform and can be combined with existing Software Defined Radio (SDR) implementations to
 * enable shared code emulation.
 *
 * Visit the <a href="https://github.com/adjacentlink/emane/wiki">EMANE Wiki</a> for information on
 * using and deploying %EMANE.
 *
 * <hr>
 * <h2>Contents</h2>
 * - <a href="#Emulator Plugins">Emulator Plugins</a>
 * - <a href="#Common Component Interfaces">Common Component Interfaces</a>
 * - <a href="#Emulator Physical Layer">Emulator Physical Layer</a>
 * - <a href="#FAQ">Developer FAQ</a>
 *
 * <hr>
 * <a name="Emulator Plugins"><h2>Emulator Plugins</h2></a>
 *  %EMANE supports 4 types of plugins:
 *    -# @ref RadioModelPlugin "Radio Models" emulate the lowest layers of a waveform. Functionality
 *       typically includes: Channel access protocols (TDMA, CSMA/CA, FDMA, CDMA) and QoS features
 *       (Queuing, Acknowledgments, Retries, Fragmentation, Segmentation).\n
 *        - The %EMANE distribution contains two Radio Models:
 *            - @ref EMANE::Models::RFPipe::MACLayer "RF Pipe Model"
 *            - @ref EMANE::Models::IEEE80211ABG::MACLayer "IEEE 802.11abg Model"
              .
 *        - The distribution contains one Utility Model
 *            - @ref EMANE::Models::CommEffect::Shim::Shim "Comm Effect Model"\n
 *             A Utility Model is not a Radio Model. It uses the features of
 *             the emulator to apply effects to traffic without specific notions 
 *             related to wireless communication.
 *            .
 *        - The distribution contains one @a Hello @a World Model:
 *            - @ref EMANE::Models::Bypass::MACLayer::MACLayer "Bypass Model"\n
 *             A simple model that does not do much of anything other than implement all the required
 *             plugin methods.
 *            .
 *    -# @ref TransportBoundaryPlugin "Application/Emulation Transport Boundaries" manage the
 *       messaging between applications running outside the emulator and the emulation stack.
 *       Transport boundaries create the entry/exit mechanism used to get IP or non-IP traffic in
 *       to and out of the emulator.
 *       - The %EMANE distribution contains two transports:
 *           - @ref EMANE::Transports::Virtual::VirtualTransport "Virtual Transport"
 *           - @ref EMANE::Transports::Raw::RawTransport "Raw Transport"
 *
 *    -# @ref EventGeneratorPlugin "Event Generators" create scenario events that are delivered to
 *       NEMs in order change environmental characteristics. Events are delivered opaquely to
 *       registered radio model instances so individual radio models may require their own
 *       specialized events. Whenever possible we advocate reusing %EMANE standard events over
 *       creating new events that do the same thing.\n
 *       - The %EMANE distribution contains one event generator:
 *           - @ref EMANE::Generators::EEL::Generator "Emulation Event Log (EEL) Generator"\n
 *           See http://downloads.pf.itd.nrl.navy.mil/docs/mnmtools/EmulationScriptSchemaDescription.pdf\n
 *           A generator that creates all of the %EMANE distribution events from flat text files:
 *             - @ref EMANE::Events::AntennaProfileEvent "AntennaProfileEvent"
 *             - @ref EMANE::Events::CommEffectEvent "CommEffectEvent"
 *             - @ref EMANE::Events::LocationEvent "LocationEvent"
 *             - @ref EMANE::Events::PathlossEvent "PathlossEvent"
 *
 *    -# @ref EventAgentPlugin "Event Agents" turn %EMANE events into usable forms suitable for
 *       processing by applications outside of the emulator.
 *       - The %EMANE distribution contains one agent:
 *           - @ref EMANE::Agents::GPSDLocation::Agent "GPSD Location Agent"\n
 *             An agent that converts location events into NMEA strings and writes them
 *             out to a pseudo terminal to emulate an attached GPS receiver.
 *           .
 *       .
 *    .
 * <hr> 
 * <a name="Common Component Interfaces"><h2>Common Component Interfaces</h2></a>
 *   Each type of plugin derives from its own specific base class but they all share certain class interfaces that are used to manage plugin life cycles and framework interaction.
 *    - @ref EMANE::Component "Component Interface" used to transition plugin states.\n
 *    - @ref EMANE::Buildable "Buildable Interface" used to associate a plugin with a unique application instance id.
 *    - @ref EMANE::PlatformServiceUser "Platform Service User Interface" used to give a plugin access to general emulator services.
 *    - @ref EMANE::RunningStateMutable "Running State Mutable Interface" used to request running-state configuration changes.
 *
 *  Each plugin has access to general services provided by the emulator.
 *   - @ref ConfigurationService "Configuration Service" used to register plugin configuration items.
 *   - @ref EventService "Event Service" used to register events of interest and to send events.
 *   - @ref LogService "Log Service" used to emit log messages.
 *   - @ref StatisticService "Statistic Service" used to create numeric and non numeric statistics and statistic tables which are accessible from outside the emulator.
 *   - @ref TimerService "Timer Service" used to schedule timed events.
 *
 * <hr>
 * <a name="Emulator Physical Layer"><h2>Emulator Physical Layer</h2></a>
 *
 * The @ref EmulatorPhysicalLayer "emulator physical layer" provides the following functionality:
 *  - Propagation Model Support
 *    - 2-Ray
 *    - Freespace
 *    - Precomputed
 *  - Receive Power Calculation
 *  - Antenna Gain Support
 *  - Noise Processing
 *  - Frequency Diversity
 *  - Collaborative Transmission
 *  - @ref RadioModelInterfacing "Radio Model Interface Support"
 *
 *  Radio Models can access per frequency noise information tracked by the emulator physical layer
 *  using the @ref SpectrumService "Spectrum Service".
 *
 * <hr>
 * <a name="FAQ"><h2>Developer FAQ</h2></a>
 * -# <strong>Can you create additional threads inside a Radio Model or Utility Model?</strong>\n
 * You can create additional threads but you might want to see if your design can be modified to not
 * require them. All %NEM layers are assigned a dedicated @ref EMANE::NEMQueuedLayer "functor queue"
 * thread. Each of the running-state plugin API messages are converted to a function object and
 * placed on the functor queue of the receiving %NEM layer for processing. The receiving NEM layer's
 * functor queue thread works each message off sequentially, preserving the API message order. You
 * can get a lot of miles out of using the functor queue in conjunction with the TimerService for
 * delayed processing, including not needing synchronization objects since the functor queue is
 * single threaded.\n
 * The following methods are enqueued on the functor queue for processing:
 *   - @ref EMANE::EventServiceUser::processEvent "processEvent"
 *   - @ref EMANE::DownstreamTransport::processDownstreamPacket "processDownstreamPacket"
 *   - @ref EMANE::DownstreamTransport::processDownstreamControl "processDownstreamControl"
 *   - @ref EMANE::RunningStateMutable::processConfiguration "processConfiguration"
 *   - @ref EMANE::TimerServiceUser::processTimedEvent "processTimedEvent"
 *   - @ref EMANE::UpstreamTransport::processUpstreamPacket "processUpstreamPacket"
 *   - @ref EMANE::UpstreamTransport::processUpstreamControl "processUpstreamControl"
 *
 * -# <strong>Why should you not block in @ref EMANE::EventServiceUser::processEvent "processEvent",
 * @ref EMANE::DownstreamTransport::processDownstreamPacket "processDownstreamPacket",
 * @ref EMANE::DownstreamTransport::processDownstreamControl "processDownstreamControl",
 * @ref EMANE::RunningStateMutable::processConfiguration "processConfiguration",
 * @ref EMANE::TimerServiceUser::processTimedEvent "processTimedEvent",
 * @ref EMANE::UpstreamTransport::processUpstreamPacket "processUpstreamPacket" and
 * @ref EMANE::UpstreamTransport::processUpstreamControl "processUpstreamControl"?</strong>\n
 * Since those calls are serially executed by the NEM layer's functor queue, blocking will delay
 * your ability to handle the next message. Instead you should look at using the @ref TimerService
 * "Timer Service".
 *
 * -# <strong>Should you use the emulator physical layer when developing your own radio model?</strong>\n
 * Yes.  While it is possible to write your own physical layer plugin, it is strongly discouraged. The
 * emulator physical layer provides features that are required by most wireless radio models and will
 * significantly reduce your development and test  cycles.  In addition, it minimizes compatibility
 * issues with current and future versions of %EMANE.  If your radio model requires features within
 * the emulator physical layer that are not currently supported, feel free to contact us to discuss
 *  potential short and long term implementation solutions.
 *
 * -# <strong>Should you use the @ref EMANE::Transports::Virtual::VirtualTransport
 * "VirtualTransport" or the @ref EMANE::Transports::Raw::RawTransport "RawTransport" when building
 * a shared code radio model for use with the upper layers of a Software Defined Radio (SDR)?</strong>\n
 * No, you should not use either. When connecting SDR waveforms to %EMANE, custom transports should
 * be developed and embedded within the SDR at the Modem Hardware Abstraction Layer (MHAL). %EMANE
 * provides application level APIs to make developing a custom transport very easy. If you are
 * using either the @ref EMANE::Transports::Virtual::VirtualTransport "VirtualTransport" or the @ref
 * EMANE::Transports::Raw::RawTransport "RawTransport" to simply pass messages between your radio
 * model plugin and your SDR code, you are doing something wrong. See @ref
 * EMANE::Application::TransportBuilder "TransportBuilder" for details, below is sample code for
 * embedding a transport in your own application using @a libemane.\n
 * @code
 * #include <emane/application/logger.h>
 * #include <emane/application/transportbuilder.h>
 *
 * ...
 *
 * // create and EMANE logger and set the initial level
 * EMANE::Application::Logger logger;
 *    
 * logger.setLogLevel(EMANE::DEBUG_LEVEL);
 *     
 * // create a transport builder
 * EMANE::Application::TransportBuilder transportBuilder;
 *     
 * // create an instance of YOUR transport
 * MyWaveform::Transport * pTransport = 
 *   transportBuilder.buildTransport<MyWaveform::Transport>(id, // nem id
 *     {{"yourconfigparamname",{"yourconfigparamvalue"}}}); // EMANE::ConfigurationUpdateRequest
 *     
 * // ownership will be transferred to TransportAdapter
 * std::unique_ptr<EMANE::Transport> ptr{pTransport}; 
 *     
 * EMANE::Application::TransportAdapters adapters;
 *     
 * adapters.push_back(transportBuilder.buildTransportAdapter(ptr, 
 *                                                           {
 *                                                             {"platformendpoint",{"localhost:8181"}},
 *                                                             {"transportendpoint",{"localhost:8281"}}
 *                                                            }
 *                                                            ));
 *     
 * // create application instance UUID 
 * uuid_t uuid;
 * uuid_generate(uuid);
 *
 * // create the transport manager and initialize with an empty EMANE::ConfigurationUpdateRequest
 * // ownership of adapters transfered to the TransportManager
 * auto pTransportManager = 
 *    transportBuilder.buildTransportManager(uuid,adapters,{});
 *     
 * // start the adapter and transport
 * pTransportManager->start();
 *     
 * // post-start the adapter and transport
 * pTransportManager->postStart();
 *
 * ... all the real work
 *
 * // stop the adapter and transport
 * pTransportManager->stop();
 *     
 * // destroy the adapter and transport
 * pTransportManager->destroy();
 * @endcode
 *
 * -# <strong>Why must @ref EMANE::StatisticNumeric "StatisticNumeric", @ref
 * EMANE::StatisticNonNumeric "StatisticNonNumeric" and @ref EMANE::StatisticTable
 * "StatisticTable" component members be pointers?</strong>\n
 * Statistics and statistic tables are owned by the emulator infrastructure. You declare members
 * to be pointers to the appropriate statistic type and then in your component's @ref
 * EMANE::Component::initialize  "initialize" method you must register each statistic and statistic
 * table using the @ref EMANE::StatisticRegistrar "StatisticRegistrar". The registration methods
 * will return a @a borrowed reference (you don't own it, you don't clean it up) to your statistic
 * or statistic table. See @ref StatisticService "Statistic Service" for details.
 *
 * -# <strong>Does the duration of a %DownstreamPacket need to hold true to the payload length and
 * datarate?</strong>\n
 * No, you can set packet duration to whatever makes sense. The emulator physical layer is not aware
 * of the datarate. If desired, you can add metadata to DownstreamPackets but maintain the @a
 * duration dictated by the radio model channel access protocol (i.e. RTS/CTS etc.). See @ref
 * RadioModelInterfacing "Radio Model Interfacing" for more information.
 */
