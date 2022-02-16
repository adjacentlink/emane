/*
 * Copyright (c) 2013,2016,2021 - Adjacent Link LLC, Bridgewater,
 *  New Jersey
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
                 const TimePoint & txTime,
                 const FrequencyGroups & frequencyGroups,
                 const Antennas & transmitAntennas,
                 const Transmitters & transmitters,
                 const std::pair<FilterData,bool> & optionalFilterData):
    registrationId_{registrationId},
    u16SubId_{u16SubId},
    u16SequenceNumber_{u16SequenceNumber},
    txTime_{txTime},
    frequencyGroups_{frequencyGroups},
    transmitAntennas_{transmitAntennas},
    transmitters_{transmitters},
    optionalFilterData_{optionalFilterData}{}

  RegistrationId getRegistrationId() const
  {
    return registrationId_;
  }

  std::uint16_t getSubId() const
  {
    return u16SubId_;
  }

  const std::pair<FilterData,bool> & getOptionalFilterData() const
  {
    return optionalFilterData_;
  }

  const TimePoint & getTxTime() const
  {
    return txTime_;
  }

  std::uint16_t getSequenceNumber() const
  {
    return u16SequenceNumber_;
  }

  const FrequencyGroups & getFrequencyGroups() const
  {
    return frequencyGroups_;
  }

  const Antennas & getTransmitAntennas() const
  {
    return transmitAntennas_;
  }

  const Transmitters & getTransmitters() const
  {
    return transmitters_;
  }

private:
  RegistrationId registrationId_;
  std::uint16_t u16SubId_;
  std::uint16_t u16SequenceNumber_;
  TimePoint txTime_;
  FrequencyGroups frequencyGroups_;
  Antennas transmitAntennas_;
  Transmitters transmitters_;
  std::pair<FilterData,bool> optionalFilterData_;
};

EMANE::CommonPHYHeader::CommonPHYHeader(UpstreamPacket & pkt)
{
  if(pkt.length() >= sizeof(decltype(pkt.stripLengthPrefixFraming())))
    {
      std::uint16_t u16HeaderLength{pkt.stripLengthPrefixFraming()};

      if(pkt.length() >= u16HeaderLength)
        {
          EMANEMessage::CommonPHYHeader msg;

          if(!msg.ParseFromArray(pkt.get(),u16HeaderLength))
            {
              throw SerializationException("unable to deserialize CommonPHYHeader");
            }

          Transmitters transmitters{};

          for(const auto & transmitter : msg.transmitters())
            {
              transmitters.push_back({static_cast<NEMId>(transmitter.nemid()),
                  transmitter.powerdbm()});
            }

          FrequencyGroups frequencyGroups{};

          for(const auto & group : msg.frequencygroups())
            {
              FrequencySegments frequencySegments{};

              for(const auto & segment : group.frequencysegments())
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
              frequencyGroups.push_back(frequencySegments);
            }

          RegistrationId registrationId{static_cast<RegistrationId>(msg.registrationid())};
          std::uint16_t u16SubId{static_cast<std::uint16_t>(msg.subid())};
          std::uint16_t u16SequenceNumber{static_cast<std::uint16_t>(msg.sequencenumber())};
          TimePoint txTime{static_cast<Microseconds>(msg.txtimemicroseconds())};
          std::pair<FilterData,bool> optionalFilterData{{},msg.has_filterdata()};

          if(optionalFilterData.second)
            {
              optionalFilterData.first = msg.filterdata();
            }

          Antennas transmitAntennas{};

          for(const auto & transmitAntenna : msg.transmitantennas())
            {
              if(transmitAntenna.has_fixedgaindbi())
                {
                  auto antenna = Antenna::createIdealOmni(transmitAntenna.antennaindex(),
                                                          transmitAntenna.fixedgaindbi());

                  antenna.setFrequencyGroupIndex(static_cast<FrequencyGroupIndex>(transmitAntenna.frequencygroupindex()));

                  antenna.setBandwidthHz(transmitAntenna.bandwidthhz());

                  if(transmitAntenna.has_spectralmaskindex())
                    {
                      antenna.setSpectralMaskIndex(transmitAntenna.spectralmaskindex());
                    }

                  transmitAntennas.push_back(std::move(antenna));
                }
              else
                {
                  if(transmitAntenna.has_pointing())
                    {
                      const auto & pointing = transmitAntenna.pointing();

                      auto antenna = Antenna::createProfileDefined(transmitAntenna.antennaindex(),
                                                                   {static_cast<AntennaProfileId>(pointing.profileid()),
                                                                    pointing.azimuthdegrees(),
                                                                    pointing.elevationdegrees()});

                      antenna.setFrequencyGroupIndex(static_cast<FrequencyGroupIndex>(transmitAntenna.frequencygroupindex()));

                      antenna.setBandwidthHz(transmitAntenna.bandwidthhz());

                      if(transmitAntenna.has_spectralmaskindex())
                        {
                          antenna.setSpectralMaskIndex(transmitAntenna.spectralmaskindex());
                        }

                      transmitAntennas.push_back(std::move(antenna));
                    }
                  else
                    {
                      auto antenna = Antenna::createProfileDefined(transmitAntenna.antennaindex());

                      antenna.setFrequencyGroupIndex(static_cast<FrequencyGroupIndex>(transmitAntenna.frequencygroupindex()));

                      antenna.setBandwidthHz(transmitAntenna.bandwidthhz());

                      if(transmitAntenna.has_spectralmaskindex())
                        {
                          antenna.setSpectralMaskIndex(transmitAntenna.spectralmaskindex());
                        }

                      transmitAntennas.push_back(std::move(antenna));
                    }
                }
            }

          pImpl_.reset(new Implementation{registrationId,
                                          u16SubId,
                                          u16SequenceNumber,
                                          txTime,
                                          frequencyGroups,
                                          transmitAntennas,
                                          transmitters,
                                          optionalFilterData});

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
                                        const TimePoint & txTime,
                                        const FrequencyGroups & frequencyGroups,
                                        const Antennas & transmitAntennas,
                                        const Transmitters & transmitters,
                                        const std::pair<FilterData,bool> & optionalFilterData):
  pImpl_{new Implementation{registrationId,
    u16SubId,
    u16SequenceNumber,
    txTime,
    frequencyGroups,
    transmitAntennas,
    transmitters,
    optionalFilterData}}
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

const std::pair<EMANE::FilterData,bool> &
EMANE::CommonPHYHeader::getOptionalFilterData() const
{
  return pImpl_->getOptionalFilterData();
}

const EMANE::TimePoint & EMANE::CommonPHYHeader::getTxTime() const
{
  return pImpl_->getTxTime();
}


EMANE::Durations EMANE::CommonPHYHeader::getDurations() const
{
  std::vector<EMANE::Microseconds> durations{};

  for(const auto & group : pImpl_->getFrequencyGroups())
    {
      Microseconds start = Microseconds::zero();
      Microseconds end = Microseconds::zero();

      for(const auto & segment : group)
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
      durations.push_back(end - start);
    }

  return durations;
}


std::uint16_t EMANE::CommonPHYHeader::getSequenceNumber() const
{
  return pImpl_->getSequenceNumber();
}


const EMANE::FrequencyGroups &
EMANE::CommonPHYHeader::getFrequencyGroups() const
{
  return pImpl_->getFrequencyGroups();
}


const EMANE::Antennas &
EMANE::CommonPHYHeader::getTransmitAntennas() const
{
  return pImpl_->getTransmitAntennas();
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
  msg.set_txtimemicroseconds(std::chrono::duration_cast<Microseconds>(pImpl_->getTxTime().time_since_epoch()).count());

  const auto & optionalFilterData = pImpl_->getOptionalFilterData();

  if(optionalFilterData.second)
    {
      msg.set_filterdata(optionalFilterData.first);
    }

  for(const auto & transmitter : pImpl_->getTransmitters())
    {
      auto pTransmitter = msg.add_transmitters();
      pTransmitter->set_nemid(transmitter.getNEMId());
      pTransmitter->set_powerdbm(transmitter.getPowerdBm());
    }

  for(const auto & group : pImpl_->getFrequencyGroups())
    {
      auto pGroup = msg.add_frequencygroups();

      for(const auto & segment : group)
        {
          auto pSegment = pGroup->add_frequencysegments();
          pSegment->set_frequencyhz(segment.getFrequencyHz());
          pSegment->set_offsetmicroseconds(segment.getOffset().count());
          pSegment->set_durationmicroseconds(segment.getDuration().count());

          if(segment.getPowerdBm().second)
            {
              pSegment->set_powerdbm(segment.getPowerdBm().first);
            }
        }
    }

  for(const auto & transmitAntenna : pImpl_->getTransmitAntennas())
    {
      auto pTransmitAntenna = msg.add_transmitantennas();

      pTransmitAntenna->set_antennaindex(transmitAntenna.getIndex());

      pTransmitAntenna->set_frequencygroupindex(transmitAntenna.getFrequencyGroupIndex());

      // only set the bandwidth if spectral mask is not in use
      if(transmitAntenna.getSpectralMaskIndex())
        {
          pTransmitAntenna->set_spectralmaskindex(transmitAntenna.getSpectralMaskIndex());
          pTransmitAntenna->set_bandwidthhz(0);
        }
      else
        {
          pTransmitAntenna->set_bandwidthhz(transmitAntenna.getBandwidthHz());
        }

      if(transmitAntenna.isIdealOmni())
        {
          pTransmitAntenna->set_fixedgaindbi(transmitAntenna.getFixedGaindBi().first);
        }
      else
        {
          auto pointing = transmitAntenna.getPointing();

          if(pointing.second)
            {
              auto pPointing = pTransmitAntenna->mutable_pointing();

              pPointing->set_profileid(pointing.first.getProfileId());
              pPointing->set_azimuthdegrees(pointing.first.getAzimuthDegrees());
              pPointing->set_elevationdegrees(pointing.first.getElevationDegrees());
            }
        }
    }


  std::string sSerialization;

  if(!msg.SerializeToString(&sSerialization))
    {
      throw SerializationException("unable to serialize CommonPHYHeader");
    }

  // prepend order is important
  pkt.prepend(sSerialization.c_str(),sSerialization.size());

  pkt.prependLengthPrefixFraming(sSerialization.size());
}

EMANE::Strings EMANE::CommonPHYHeader::format() const
{
  Strings sFormat{{"regid: " + std::to_string( pImpl_->getRegistrationId())},
                  {"seq: " + std::to_string(pImpl_->getSequenceNumber())},
                  {"tx time: " +  std::to_string(std::chrono::duration_cast<DoubleSeconds>(pImpl_->getTxTime().time_since_epoch()).count())}};

  int i{};

  for(const auto & group : pImpl_->getFrequencyGroups())
    {
      sFormat.push_back("freq group: " + std::to_string(i++));

      for(const auto & segment : group)
        {
          sFormat.push_back("freq: " + std::to_string(segment.getFrequencyHz()));
          sFormat.push_back("duration: " + std::to_string(segment.getDuration().count()));
          sFormat.push_back("offset: " + std::to_string(segment.getOffset().count()));
          if(segment.getPowerdBm().second)
            {
              sFormat.push_back("segment power: " + std::to_string(segment.getPowerdBm().first));
            }
        }
    }


  for(const auto & transmitAntenna : pImpl_->getTransmitAntennas())
    {
      sFormat.push_back("antenna: " + std::to_string(transmitAntenna.getIndex()));
      sFormat.push_back("freq index: " + std::to_string(transmitAntenna.getFrequencyGroupIndex()));
      sFormat.push_back("bandwidth: " + std::to_string(transmitAntenna.getBandwidthHz()));
      sFormat.push_back("mask: " + std::to_string(transmitAntenna.getSpectralMaskIndex()));

      if(transmitAntenna.isIdealOmni())
        {
          sFormat.push_back("fixed gain: " + std::to_string(transmitAntenna.getFixedGaindBi().second));
        }
      else
        {
          const auto & pointing = transmitAntenna.getPointing();

          if(pointing.second)
            {
              sFormat.push_back("profile id: " + std::to_string(pointing.first.getProfileId()));
              sFormat.push_back("azimuth: " + std::to_string(pointing.first.getAzimuthDegrees()));
              sFormat.push_back("elevation: " + std::to_string(pointing.first.getElevationDegrees()));
            }
        }
    }

  for(const auto & transmitter : pImpl_->getTransmitters())
    {
      sFormat.push_back("src: " + std::to_string(transmitter.getNEMId()));
      sFormat.push_back("transmitter power: " + std::to_string(transmitter.getPowerdBm()));
    }

  const auto & optionalFilterData = pImpl_->getOptionalFilterData();

  if(optionalFilterData.second)
    {
      sFormat.push_back("fitler data bytes: " + std::to_string(optionalFilterData.first.size()));
    }

  return sFormat;
}
