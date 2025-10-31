

//                                     ===================================  FUNCTION DECLARATIONS ===================================

void sendTelemetry();






//                                           ===================================  TIMERS  ===================================









void setup() {




}

void loop() {



}


//                                           ===================================  SEND TELEMETRY  ===================================

sendTelemetry() {


  



  Serial.print(TeamID); Serial.print(",");
  Serial.print(TeamID); Serial.print(",");
  Serial.print(stateNames[flightState]); Serial.print(",");
  Serial.print(altitude, 2); Serial.print(",");
  Serial.print(temperature, 1); Serial.print(",");
  Serial.print(velocity, 2); Serial.print(",");
  Serial.print(acceleration_x, 2); Serial.print(",");
  Serial.print(acceleration_y, 2); Serial.print(",");
  Serial.print(acceleration_z, 2); Serial.print(",");
  Serial.print(compassHeading); Serial.print(",");
  Serial.print(gpsLatitude, 5); Serial.print(",");
  Serial.print(gpsLongitude, 5); Serial.print(",");
  Serial.print(batteryVoltage, 2); Serial.print(",");
  Serial.print(gyroRoll, 1); Serial.print(",");
  Serial.print(gyroPitch, 1); Serial.print(",");
  Serial.print(gyroYaw, 1); Serial.print(",");
  Serial.print(flightTime); Serial.print(",,");
  Serial.print(gpsSatellites); Serial.print(",");
  Serial.print(aerobrakeOverride ? "MANUAL" : "AUTO"); Serial.print(",");
  Serial.print(aerobrakeDeployed ? "DEPLOYED" : "RETRACTED"); Serial.print(",");
  Serial.print(getFlightLedCode()); Serial.print(",");
  Serial.print(pressurePa, 1); Serial.print(",");
  Serial.print(initialVelocity, 1); Serial.print(",");
  Serial.print(acceleration_highest, 2); Serial.print(",");
  Serial.print(peakAltitude, 2); Serial.println();










}


