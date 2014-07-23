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

#include "eventtablepublisher.h"

EMANE::EventTablePublisher::EventTablePublisher(NEMId nemId):
  nemId_{nemId}{};

void EMANE::EventTablePublisher::registerStatistics(StatisticRegistrar & statisticRegistrar)
{
  /** [statisticservice-registertable-snippet] */
  pLocationTable_ =
    statisticRegistrar.registerTable<NEMId>("LocationEventInfoTable", 
      {"NEM","Latitude","Longitude","Altitude","Pitch","Roll","Yaw","Azimuth","Elevation","Magnitude"},
      StatisticProperties::NONE,
      "Shows the location event information received");

  pPathlossTable_ =
    statisticRegistrar.registerTable<NEMId>("PathlossEventInfoTable", 
      {"NEM","Forward Pathloss","Reverse Pathloss"},
      StatisticProperties::NONE,
      "Shows the precomputed pathloss information received");

  pAntennaProfileTable_ =
    statisticRegistrar.registerTable<NEMId>("AntennaProfileEventInfoTable", 
      {"NEM","Antenna Profile","Antenna Azimuth","Antenna Elevation"},
      StatisticProperties::NONE,
      "Shows the antenna profile information received");
  /** [statisticservice-registertable-snippet] */
}

void EMANE::EventTablePublisher::update(const Events::Locations & locations)
{
  for(const auto & location : locations)
    {
      auto targetNEM = location.getNEMId();
       
      if(locationNEMSet_.find(targetNEM) == locationNEMSet_.end())
        {
          auto position = location.getPosition();
           
          auto optionalOrientation = location.getOrientation();

          auto optionalVelocity = location.getVelocity();
           
          pLocationTable_->addRow(targetNEM,{Any{targetNEM},
                Any{position.getLatitudeDegrees()},
                  Any{position.getLongitudeDegrees()},
                    Any{position.getAltitudeMeters()},
                      Any{optionalOrientation.first.getPitchDegrees()},
                        Any{optionalOrientation.first.getRollDegrees()},
                          Any{optionalOrientation.first.getYawDegrees()},
                            Any{optionalVelocity.first.getAzimuthDegrees()},
                              Any{optionalVelocity.first.getElevationDegrees()},
                                Any{optionalVelocity.first.getMagnitudeMetersPerSecond()}});

          locationNEMSet_.insert(targetNEM);
        }
      else
        {
          auto position = location.getPosition();
           
          auto optionalOrientation = location.getOrientation();
           
          auto optionalVelocity = location.getVelocity();
           
          pLocationTable_->setCell(targetNEM,1,Any{position.getLatitudeDegrees()});
          pLocationTable_->setCell(targetNEM,2,Any{position.getLongitudeDegrees()});
          pLocationTable_->setCell(targetNEM,3,Any{position.getAltitudeMeters()});

          if(optionalOrientation.second)
            {
              pLocationTable_->setCell(targetNEM,4,Any{optionalOrientation.first.getPitchDegrees()});
              pLocationTable_->setCell(targetNEM,5,Any{optionalOrientation.first.getRollDegrees()});
              pLocationTable_->setCell(targetNEM,6,Any{optionalOrientation.first.getYawDegrees()});
            }

          if(optionalVelocity.second)
            {
              pLocationTable_->setCell(targetNEM,7,Any{optionalVelocity.first.getAzimuthDegrees()});
              pLocationTable_->setCell(targetNEM,8,Any{optionalVelocity.first.getElevationDegrees()});
              pLocationTable_->setCell(targetNEM,9,Any{optionalVelocity.first.getMagnitudeMetersPerSecond()});
               
            }
        }
    }
}

void EMANE::EventTablePublisher::update(const Events::Pathlosses & pathlosses)
{
  for(const auto & pathloss : pathlosses)
    {
      auto targetNEM = pathloss.getNEMId();
      
      std::vector<Any> row{Any{targetNEM},
          Any{pathloss.getForwardPathlossdB()},
            Any{pathloss.getReversePathlossdB()}};
      
      if(pathlossNEMSet_.find(targetNEM) == pathlossNEMSet_.end())
        {
          pPathlossTable_->addRow(targetNEM,row);
          
          pathlossNEMSet_.insert(targetNEM);
        }
      else
        {
          pPathlossTable_->setRow(targetNEM,row);
        }
    }
}

void EMANE::EventTablePublisher::update(const AntennaProfiles & profiles)
{
  
  for(const auto & profile : profiles)
    {
      auto targetNEM = profile.getNEMId();
      
      std::vector<Any> row{Any{targetNEM},
          Any{profile.getAntennaProfileId()},
            Any{profile.getAntennaAzimuthDegrees()},
              Any{profile.getAntennaElevationDegrees()}};
      
      if(antennaProfileNEMSet_.find(targetNEM) == antennaProfileNEMSet_.end())
        {
          pAntennaProfileTable_->addRow(targetNEM,row);
          
          antennaProfileNEMSet_.insert(targetNEM);
        }
      else
        {
          pAntennaProfileTable_->setRow(targetNEM,row);
        }
    }
}
