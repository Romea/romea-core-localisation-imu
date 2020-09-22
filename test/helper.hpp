#include "romea_gps/nmea/GGAFrame.hpp"
#include "romea_gps/nmea/RMCFrame.hpp"

romea::GGAFrame minimalGoodGGAFrame()
{
  romea::GGAFrame frame;
  frame.talkerId=romea::TalkerId::GN;
  frame.longitude=romea::Longitude(0.03);
  frame.latitude=romea::Latitude(0.7854);
  frame.geoidHeight=400.8;
  frame.altitudeAboveGeoid=53.3;
  frame.horizontalDilutionOfPrecision=1.2;
  frame.numberSatellitesUsedToComputeFix=12;
  frame.fixQuality = romea::FixQuality::RTK_FIX;
  frame.horizontalDilutionOfPrecision=1.2;
  frame.dgpsCorrectionAgeInSecond=2.5;
  frame.dgpsStationIdNumber=1;
  return frame;
}

romea::RMCFrame minimalGoodRMCFrame()
{
  romea::RMCFrame frame;
  frame.talkerId=romea::TalkerId::GP;
  frame.speedOverGroundInMeterPerSecond=3.2;
  frame.trackAngleTrue=1.54;
  frame.magneticDeviation=0.0378;
  return frame;
}
