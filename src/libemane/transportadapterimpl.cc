/*
 * Copyright (c) 2013-2017 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2011-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include "transportadapterimpl.h"
#include "logservice.h"
#include "boundarymessagemanagerexception.h"

#include "emane/configureexception.h"
#include "emane/startexception.h"

EMANE::Application::TransportAdapterImpl::TransportAdapterImpl(NEMId nemId):
  TransportAdapter{nemId},
  BoundaryMessageManager{nemId},
  id_{nemId}{}

EMANE::Application::TransportAdapterImpl::~TransportAdapterImpl(){}

void EMANE::Application::TransportAdapterImpl::setTransport(std::unique_ptr<NEMLayer> & pTransport)
{
  pTransport_ = std::move(pTransport);
  setUpstreamTransport(pTransport_.get());
  pTransport_->setDownstreamTransport(this);
}

void EMANE::Application::TransportAdapterImpl::initialize(Registrar & registrar)
{
  auto & configRegistrar = registrar.configurationRegistrar();

  configRegistrar.registerNonNumeric<INETAddr>("platformendpoint",
                                               ConfigurationProperties::REQUIRED,
                                               {},
                                               "IPv4 or IPv6 NEM Platform Service endpoint.");

  configRegistrar.registerNonNumeric<INETAddr>("transportendpoint",
                                               ConfigurationProperties::REQUIRED,
                                               {},
                                               "IPv4 or IPv6 Transport endpoint.");

  configRegistrar.registerNonNumeric<std::string>("protocol",
                                                  EMANE::ConfigurationProperties::DEFAULT,
                                                  {"udp"},
                                                  "Defines the protocl used for communictation:"
                                                  " udp or tcp.",
                                                  1,
                                                  1,
                                                  "^(udp|tcp)$");
}

void EMANE::Application::TransportAdapterImpl::configure(const ConfigurationUpdate & update)
{
  for(const auto & item : update)
    {
      if(item.first == "platformendpoint")
        {
          platformEndpointAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "TRANS %03hu TransportAdapterImpl::configure %s: %s",
                                  id_,
                                  item.first.c_str(),
                                  platformEndpointAddr_.str().c_str());

        }
      else if(item.first == "transportendpoint")
        {
          transportEndpointAddr_ = item.second[0].asINETAddr();

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "TRANS %03hu TransportAdapterImpl::configure %s: %s",
                                  id_,
                                  item.first.c_str(),
                                  transportEndpointAddr_.str().c_str());
        }
      else if(item.first == "protocol")
        {
          std::string sProtocol{item.second[0].asString()};

          protocol_ = sProtocol == "udp" ?
            Protocol::PROTOCOL_UDP :
            Protocol::PROTOCOL_TCP_CLIENT;

          LOGGER_STANDARD_LOGGING(*LogServiceSingleton::instance(),
                                  INFO_LEVEL,
                                  "NEM  %03hu TransportAdapterImpl::configure %s: %s",
                                  id_,
                                  item.first.c_str(),
                                  sProtocol.c_str());
        }
      else
        {
          throw makeException<ConfigureException>("TransportAdapterImpl: "
                                                  "Unexpected configuration item %s",
                                                  item.first.c_str());
        }
    }
}

void EMANE::Application::TransportAdapterImpl::start()
{
  if(!pTransport_)
    {
      throw makeException<StartException>("TransportAdapterImpl transport not set for NEM %hu",
                                          id_);
    }

  try
    {
      open(transportEndpointAddr_, platformEndpointAddr_,protocol_);
    }
  catch(BoundaryMessageManagerException & exp)
    {
      throw StartException(exp.what());
    }

  pTransport_->start();
}

void EMANE::Application::TransportAdapterImpl::postStart()
{
  pTransport_->postStart();
}

void EMANE::Application::TransportAdapterImpl::stop()
{
  pTransport_->stop();
}

void EMANE::Application::TransportAdapterImpl::destroy()
  throw()
{
  pTransport_->destroy();
}

void EMANE::Application::TransportAdapterImpl::processDownstreamPacket(EMANE::DownstreamPacket & pkt,
                                                                       const EMANE::ControlMessages & msgs)
{
  sendPacketMessage(pkt.getPacketInfo(),
                    pkt.getVectorIO(),
                    pkt.length(),
                    msgs);
}

void
EMANE::Application::TransportAdapterImpl::processDownstreamControl(const EMANE::ControlMessages & msgs)
{
  sendControlMessage(msgs);
}

void EMANE::Application::TransportAdapterImpl::doProcessPacketMessage(const PacketInfo & packetInfo,
                                                                      const void * pPacketData,
                                                                      size_t packetLength,
                                                                      const ControlMessages & msgs)
{
  UpstreamPacket pkt(packetInfo,
                     pPacketData,
                     packetLength);

  pTransport_->processUpstreamPacket(pkt,msgs);
}

void EMANE::Application::TransportAdapterImpl::doProcessControlMessage(const ControlMessages & msgs)
{
  pTransport_->processUpstreamControl(msgs);
}
