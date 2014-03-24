/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2010 - DRS CenGen, LLC, Columbia, Maryland
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

#include "emane/flowcontrolmanager.h"

#include <ace/Guard_T.h>
#include <ace/Thread_Mutex.h>

class EMANE::FlowControlManager::Implementation
{
public:
Implementation(EMANE::DownstreamTransport & transport):
  rTransport_(transport),
  u16TokensAvailable_{},
  u16TotalTokensAvailable_{},
  u16ShadowTokenCount_{},
  bAckPending_{true}{}
  
  ~Implementation(){}

  void start(std::uint16_t u16TotalTokensAvailable)
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);
    
    u16TotalTokensAvailable_ = u16TotalTokensAvailable;
    u16TokensAvailable_      = u16TotalTokensAvailable;
    
    sendFlowControlResponseMessage();
  }
  
  void stop()
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);
    
    u16TotalTokensAvailable_ = 0;
    u16TokensAvailable_      = 0;
    u16ShadowTokenCount_     = 0;
  }

  bool addToken(std::uint16_t u16Tokens)
  {
  ACE_Guard<ACE_Thread_Mutex> m(mutex_);

  // get add tokens status
  const bool bStatus = ((u16TokensAvailable_ + u16Tokens) <= u16TotalTokensAvailable_);

  // have room to add all requested tokens
  if(bStatus == true)
    { 
      // update tokens available
      u16TokensAvailable_ += u16Tokens;
    }

  // check shadow count, send update if tokens are available
  if((u16ShadowTokenCount_ == 0) && (u16TokensAvailable_ > 0))
    {

      sendFlowControlResponseMessage();
    }

  return bStatus;
  }

  bool removeToken()
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);
    
    bool bStatus;
    
    // ack pending, no action
    if(bAckPending_ == true)
      {
        bStatus = false;
      }
    else 
      {
        --u16ShadowTokenCount_;
        
        // no tokens available
        if(u16TokensAvailable_ == 0)
          {
            bStatus = false;
        }
        else
          {
            // decrement tokens available
            --u16TokensAvailable_;
            
            bStatus = true;
          }
      }

    return bStatus;
  }

  void processFlowControlMessage(const Controls::FlowControlControlMessage * pMsg)
  {
    ACE_Guard<ACE_Thread_Mutex> m(mutex_);
    
    if(!bAckPending_)
      {
        // upstream layer start or restart condition
        // detected
        sendFlowControlResponseMessage();
      }
    else
      {
        // a proper acknowledgment echos the token account will allow
        // traffic to flow
        if(pMsg->getTokens() == u16TokensAvailable_)
          {
            bAckPending_ = false;
          }
        else
          {
            // upstream layer missed the initial token message
            // so we received a message with an incorrect number token
            // count - resend a reponse
            sendFlowControlResponseMessage();
          }
      }
  }

private:

  DownstreamTransport & rTransport_;

  ACE_Thread_Mutex mutex_;
  
  std::uint16_t u16TokensAvailable_;
  
  std::uint16_t u16TotalTokensAvailable_;
  
  std::uint16_t u16ShadowTokenCount_;
  
  bool bAckPending_;

  // precondition - mutex is locked
  void sendFlowControlResponseMessage()
  {
    rTransport_.sendUpstreamControl({Controls::FlowControlControlMessage::create(u16TokensAvailable_)});
                                    
    u16ShadowTokenCount_ = u16TokensAvailable_;
    
    bAckPending_ = true;
  }
};


EMANE::FlowControlManager::FlowControlManager(DownstreamTransport & transport):
  pImpl_{new Implementation{transport}}
{}

EMANE::FlowControlManager::~FlowControlManager(){}

void EMANE::FlowControlManager::start(std::uint16_t u16TotalTokensAvailable)
{
  pImpl_->start(u16TotalTokensAvailable);
}

void EMANE::FlowControlManager::stop()
{
  pImpl_->stop();
}

bool EMANE::FlowControlManager::addToken(std::uint16_t u16Tokens)
{
  return pImpl_->addToken(u16Tokens);
}

bool EMANE::FlowControlManager::removeToken()
{
  return pImpl_->removeToken();
}

void EMANE::FlowControlManager::processFlowControlMessage(const Controls::FlowControlControlMessage * pMsg)
{
  pImpl_->processFlowControlMessage(pMsg);
}

