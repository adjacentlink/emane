/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANENEMLAYERSTATE_HEADER_
#define EMANENEMLAYERSTATE_HEADER_

#include "emane/nemlayer.h"

namespace EMANE
{
  class NEMStatefulLayer;
  class NEMLayer;
  
  /**
   * @class NEMLayerState
   *
   * @brief Encapsulated behavior associated with an NEMLayer depending
   * on the layer's current context
   */
  class NEMLayerState
  {
  public:
    virtual ~NEMLayerState() = 0;

    /**
     * Handle initialize
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param registrar Registrar reference 
     * 
     * @exception InitializeException
     *
     * @note Default implementation generates a log error
     */ 
    virtual void handleInitialize(NEMStatefulLayer * pStatefulLayer,
                                  NEMLayer * pLayer,
                                  Registrar & registrar);

    /**
     * Handle configuration
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param update Configuration update
     * 
     * @exception ConfigureException
     *
     * @note Default implementation generates a log error
     *
     */
    virtual void handleConfigure(NEMStatefulLayer * pStatefulLayer, 
                                 NEMLayer * pLayer, 
                                 const ConfigurationUpdate & update);

    /**
     * Handle start
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception StartException
     *
     * @note Default implementation generates a log error
     */
    virtual void handleStart(NEMStatefulLayer * pStatefulLayer,
                             NEMLayer * pLayer);

    
    /**
     * Handle post start
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @note Default implementation generates a log error
     */
    virtual void handlePostStart(NEMStatefulLayer * pStatefulLayer,
                                 NEMLayer * pLayer);

     /**
     * Handle stop
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @exception StopException
     *
     * @note Default implementation generates a log error
     */
    virtual void handleStop(NEMStatefulLayer * pStatefulLayer,
                            NEMLayer * pLayer);

    /**
     * Handle destroy
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * 
     * @note Default implementation generates a log error
     */
    virtual void handleDestroy(NEMStatefulLayer * pStatefulLayer,
                               NEMLayer * pLayer)
      throw();


    /**
     *  Process configuration update
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param update Reference to the configuration update
     *
     * @note Default implementation generates a log error
     */
    virtual void processConfiguration(NEMStatefulLayer * pStatefulLayer, 
                                      NEMLayer * pLayer, 
                                      const ConfigurationUpdate & update);


    /**
     *  Process downstream control
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param msgs Reference to the Control message
     *
     * @note Default implementation generates a log error
     */
    virtual void processDownstreamControl(NEMStatefulLayer * pStatefulLayer, 
                                          NEMLayer * pLayer, 
                                          const ControlMessages & msgs);

    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Downstream packet
     * @param msgs reference to the ControlMessage
     *
     * @note Default implementation generates a log error
     */ 
    virtual void processDownstreamPacket(NEMStatefulLayer * pStatefulLayer, 
                                         NEMLayer * pLayer,  
                                         DownstreamPacket & pkt,
                                         const ControlMessages & msgs);
    
    /**
     *  Process downstream packet
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param pkt Reference to the Upstream packet
     * @param msgs reference to the ControlMessage
     *
     * @note Default implementation generates a log error
     */
    virtual void processUpstreamPacket(NEMStatefulLayer * pStatefulLayer, 
                                       NEMLayer * pLayer, 
                                       UpstreamPacket & pkt,
                                       const ControlMessages & msgs);

    /**
     *  Process upstream control
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param msgs Reference to the Control message
     *
     * @note Default implementation generates a log error
     */
    virtual void processUpstreamControl(NEMStatefulLayer * pStatefulLayer, 
                                        NEMLayer * pLayer, 
                                        const ControlMessages & msgs);
    
    /**
     *  Process event
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param id Event Id
     * @param serialization Event object serialization
     *
     * @note Default implementation generates a log error
     */
    virtual void processEvent(NEMStatefulLayer * pStatefulLayer,
                              NEMLayer * pLayer,
                              const EventId & id, 
                              const Serialization & serialization);


    /**
     *  Process timed event
     *
     * @param pStatefulLayer Reference to the stateful layer
     * @param pLayer Reference to the wrapped layer
     * @param eventId Identifier corresponding to the timer being processed
     * @param expireTime The time the timer was scheduled to expire
     * @param scheduleTime The Time the timer was scheduled
     * @param fireTime Time time The time the timer actually fired
     * @param arg Opaque timed event data
     *
     * @note Default implementation generates a log error
     */
    virtual void processTimedEvent(NEMStatefulLayer * pStatefulLayer,
                                   NEMLayer * pLayer,
                                   TimerEventId eventId,
                                   const TimePoint & expireTime,
                                   const TimePoint & scheduleTime,
                                   const TimePoint & fireTime,
                                   const void * arg);
   
    /**
     * Get state name
     *
     * @return state Name of current state
     */
    std::string getStateName() const;

  protected:
    const char * pzStateName_;

    NEMLayerState(const char * pzStateName);
    
    void changeState(NEMStatefulLayer * pStatefulLayer,NEMLayerState * pState);
  };
}

#endif //EMANENEMLAYERSTATE_HEADER_
