/*
 * Copyright (c) 2013 - Adjacent Link LLC, Bridgewater, New Jersey
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

#include "emane/commonphyheader.h"
#include "commonphyheader.pb.h"

class EMANE::CommonPHYHeader::Implementation
{
public:
  Implementation(RegistrationId registrationId,
                 std::uint16_t u16SubId,
                 std::uint16_t u16SequenceNumber,
                 std::uint64_t u64BandwidthHz,
                 const TimePoint  & txTime,
                 const FrequencySegments & frequencySegments,
                 const Transmitters & transmitters,
                 const std::pair<double,bool> & optionalFixedAntennaGaindBi):
    registrationId_{registrationId},
    u16SubId_{u16SubId},
    u16SequenceNumber_{u16SequenceNumber},
    u64BandwidthHz_{u64BandwidthHz},
    txTime_{txTime},
    frequencySegments_{frequencySegments},
    transmitters_{transmitters},
    optionalFixedAntennaGaindBi_{optionalFixedAntennaGaindBi}{}
    
  RegistrationId getRegistrationId() const
  {
    return registrationId_;
  }

  std::uint16_t getSubId() const
  {
    return u16SubId_;
  }
  
const std::pair<double,bool> & getOptionalFixedAntennaGaindBi() const
  {
    return optionalFixedAntennaGaindBi_;
  }
  
  const TimePoint & getTxTime() const
  {
    return txTime_;
  }
  
  std::uint64_t getBandwidthHz() const
  {
    return u64BandwidthHz_;
  }
  
  std::uint16_t getSequenceNumber() const
  {
    return u16SequenceNumber_;
  }
  
  const FrequencySegments & getFrequencySegments() const
  {
    return frequencySegments_;
  }
  
  const Transmitters & getTransmitters() const
  {
    return transmitters_;
  }
  
private:
  RegistrationId registrationId_;
  std::uint16_t u16SubId_;
  std::uint16_t u16SequenceNumber_;
  std::uint64_t u64BandwidthHz_;
  TimePoint txTime_;
  FrequencySegments frequencySegments_;
  Transmitters transmitters_;
  std::pair<double,bool> optionalFixedAntennaGaindBi_;
};

EMANE::CommonPHYHeader::CommonPHYHeader(UpstreamPacket & pkt)
{
  if(pkt.length() >= sizeof(decltype(pkt.stripLengthPrefixFraming())))
    {
      std::uint16_t u16HeaderLength{pkt.stripLengthPrefixFraming()};

      if(pkt.length() >= u16HeaderLength)
        {
          EMANEMessage::CommonPHYHeader msg;
          
          try
            {
              if(!msg.ParseFromArray(pkt.get(),u16HeaderLength))
                {
                  throw SerializationException("unable to deserialize CommonPHYHeader");
                }
            }
          catch(google::protobuf::FatalException & exp)
            {
              throw SerializationException("unable to deserialize CommonPHYHeader");
            }
          
          using RepeatedPtrFieldTransmitter = 
            google::protobuf::RepeatedPtrField<EMANEMessage::CommonPHYHeader::Transmitter>;
          
          Transmitters transmitters{};
          
          for(const auto & repeatedTransmitter :
                RepeatedPtrFieldTransmitter(msg.transmitters()))
            {
              transmitters.push_back({static_cast<NEMId>(repeatedTransmitter.nemid()),
                    repeatedTransmitter.powerdbm()});
            }
          
          using RepeatedPtrFieldFrequencySegment = 
             google::protobuf::RepeatedPtrField<EMANEMessage::CommonPHYHeader::FrequencySegment>;
          
          FrequencySegments frequencySegments{};
          
          for(const auto & segment :
                RepeatedPtrFieldFrequencySegment(msg.frequencysegments()))
            {
              if(segment.has_powerdbm())
                {
                  frequencySegments.push_back({segment.frequencyhz(),
                        segment.powerdbm(),
                        Microseconds(segment.durationmicroseconds()),
                        Microseconds(segment.offsetmicroseconds())});
                }
              else
                {
                  frequencySegments.push_back({segment.frequencyhz(),
                        Microseconds(segment.durationmicroseconds()),
                        Microseconds(segment.offsetmicroseconds())});
                }
            }

          RegistrationId registrationId{static_cast<RegistrationId>(msg.registrationid())};
          std::uint16_t u16SubId{static_cast<std::uint16_t>(msg.subid())};
          std::uint16_t u16SequenceNumber{static_cast<std::uint16_t>(msg.sequencenumber())};
          std::uint64_t u64BandwidthHz{msg.bandwidthhz()};
          TimePoint txTime{static_cast<Microseconds>(msg.txtimemicroseconds())};
          std::pair<double,bool> optionalFixedAntennaGaindBi{0,msg.has_fixedantennagain()};

          if(optionalFixedAntennaGaindBi.second)
            {
              optionalFixedAntennaGaindBi.first = msg.fixedantennagain();
            }

          pImpl_.reset(new Implementation{registrationId,
                u16SubId,
                u16SequenceNumber,
                u64BandwidthHz,
                txTime,
                frequencySegments,
                transmitters,
                optionalFixedAntennaGaindBi});
              
        }
      else
        {
           throw SerializationException("CommonPHYHeader not large enough for header data");
        }
      
      // strip common phy header from pkt
      pkt.strip(u16HeaderLength);
    }
  else
    {
      throw SerializationException("CommonPHYHeader not large enough for header size data");
    }
}


EMANE::CommonPHYHeader::CommonPHYHeader(RegistrationId registrationId, 
                                        std::uint16_t u16SubId,
                                        std::uint16_t u16SequenceNumber,
                                        std::uint64_t u64BandwidthHz,
                                        const TimePoint & txTime,
                                        const FrequencySegments & frequencySegments,
                                        const Transmitters & transmitters,
                                        const std::pair<double,bool> & optionalFixedAntennaGaindBi):
pImpl_{new Implementation{registrationId, 
      u16SubId,
      u16SequenceNumber,
      u64BandwidthHz,
      txTime,
      frequencySegments,
      transmitters,
      optionalFixedAntennaGaindBi}}
{}


EMANE::CommonPHYHeader::CommonPHYHeader(CommonPHYHeader&& rvalue):
  pImpl_(std::move(rvalue.pImpl_))
{}

EMANE::CommonPHYHeader::~CommonPHYHeader(){}

EMANE::RegistrationId EMANE::CommonPHYHeader::getRegistrationId() const
{
  return pImpl_->getRegistrationId();
}

std::uint16_t EMANE::CommonPHYHeader::getSubId() const
{
  return pImpl_->getSubId();
}
    
const std::pair<double,bool> & EMANE::CommonPHYHeader::getOptionalFixedAntennaGaindBi() const
{
  return pImpl_->getOptionalFixedAntennaGaindBi();
}


const EMANE::TimePoint & EMANE::CommonPHYHeader::getTxTime() const
{
  return pImpl_->getTxTime();
}

    
EMANE::Microseconds EMANE::CommonPHYHeader::getDuration() const
{
  Microseconds start = Microseconds::zero();
  Microseconds end = Microseconds::zero();

  for(const auto & segment: pImpl_->getFrequencySegments())
    {
      // the offset
      const auto & offset = segment.getOffset();
      const auto & duration = segment.getDuration();
      
      // duration position
      const auto relative = offset + duration;
      
      // get the begin time
      if(offset < start)
        {
          start = offset;
        }
      
      // get the end time
      if(relative > end)
        {
          end = relative;
        }
    }

  // the total duration with gaps/overlap
  return end - start;
}

    
std::uint64_t EMANE::CommonPHYHeader::getBandwidthHz() const
{
  return pImpl_->getBandwidthHz();
}


std::uint16_t EMANE::CommonPHYHeader::getSequenceNumber() const
{
  return pImpl_->getSequenceNumber();
}


const EMANE::FrequencySegments & EMANE::CommonPHYHeader::getFrequencySegments() const
{
  return pImpl_->getFrequencySegments();
}

    
const EMANE::Transmitters & EMANE::CommonPHYHeader::getTransmitters() const
{
  return pImpl_->getTransmitters();
}
    
void EMANE::CommonPHYHeader::prependTo(DownstreamPacket & pkt) const
{
  EMANEMessage::CommonPHYHeader msg{};
  msg.set_registrationid(pImpl_->getRegistrationId());
  msg.set_subid(pImpl_->getSubId());
  msg.set_sequencenumber(pImpl_->getSequenceNumber());
  msg.set_bandwidthhz(pImpl_->getBandwidthHz());   
  msg.set_txtimemicroseconds(std::chrono::duration_cast<Microseconds>(pImpl_->getTxTime().time_since_epoch()).count());

  const auto & optionalFixedAntennaGaindBi = pImpl_->getOptionalFixedAntennaGaindBi();
  
  if(optionalFixedAntennaGaindBi.second)
    {
      msg.set_fixedantennagain(optionalFixedAntennaGaindBi.first);
    }

  for(const auto & transmitter : pImpl_->getTransmitters())
    {
      auto pTransmitter = msg.add_transmitters();
      pTransmitter->set_nemid(transmitter.getNEMId());
      pTransmitter->set_powerdbm(transmitter.getPowerdBm());
    }
  
  for(const auto & segment : pImpl_->getFrequencySegments())
    {
      auto pSegment = msg.add_frequencysegments();
      pSegment->set_frequencyhz(segment.getFrequencyHz());
      pSegment->set_offsetmicroseconds(segment.getOffset().count());
      pSegment->set_durationmicroseconds(segment.getDuration().count());

      if(segment.getPowerdBm().second)
        {
          pSegment->set_powerdbm(segment.getPowerdBm().first);
        }
    }

  std::string sSerialization;
  
  try
    {
      if(!msg.SerializeToString(&sSerialization))
        {
          throw SerializationException("unable to serialize CommonPHYHeader");
        }
    }
  catch(google::protobuf::FatalException & exp)
    {
      throw SerializationException("unable to serialize CommonPHYHeader");
    }
  
  // prepend order is important
  pkt.prepend(sSerialization.c_str(),sSerialization.size());
  
  pkt.prependLengthPrefixFraming(sSerialization.size());  
}

EMANE::Strings EMANE::CommonPHYHeader::format() const
{
  const auto & optionalFixedAntennaGaindBi =
    pImpl_->getOptionalFixedAntennaGaindBi();
  
  Strings sFormat{{"regid: " + std::to_string( pImpl_->getRegistrationId())},
      {"seq: " + std::to_string(pImpl_->getSequenceNumber())},
        {"bandwidth: " + std::to_string(pImpl_->getBandwidthHz())},
          {"fixed antenna gain: " + std::string(optionalFixedAntennaGaindBi.second ? "on" : "off")},
            {"fixed antenna gain: " + std::to_string(optionalFixedAntennaGaindBi.first)},
              {"tx time: " +  std::to_string(std::chrono::duration_cast<DoubleSeconds>(pImpl_->getTxTime().time_since_epoch()).count())}};
  
  for(const auto & segment : pImpl_->getFrequencySegments())
    {
      sFormat.push_back("freq: " + std::to_string(segment.getFrequencyHz()));
      sFormat.push_back("duration: " + std::to_string(segment.getDuration().count()));
      sFormat.push_back("offset: " + std::to_string(segment.getOffset().count()));
      if(segment.getPowerdBm().second)
        {
          sFormat.push_back("segment power: " + std::to_string(segment.getPowerdBm().first));
        }
    }

  for(const auto & transmitter : pImpl_->getTransmitters())
    {
      sFormat.push_back("src: " + std::to_string(transmitter.getNEMId()));
      sFormat.push_back("transmitter power: " + std::to_string(transmitter.getPowerdBm()));
    }

  return sFormat;
}
