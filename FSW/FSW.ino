//                                     ===================================  FUNCTION DECLARATIONS ===================================

void sendTelemetry();
void sampleData();






//                                           ===================================  TIMERS  ===================================









void setup() {




}

void loop() {






}




//                                           ===================================  SAMPLE DATA  ===================================

sampleData() {





//                                                                ---------------- Pointers ----------------

int teamID = 1093;
unsigned long missionTime;
int packetCount;
int determinedMode = currentMode ? 0 : 1;
string mode[2] = { "F", "S"}




}




//                                           ===================================  SEND TELEMETRY  ===================================

sendTelemetry() {

  //                                                            ---------------- Telemetry Pointers ----------------

  int *pTeam_ID = &teamID;
  unsigned long *pMission_time = &missionTime;
  int *pPacket_count = &packetCount;
  int *pDetermined_Mode = &determinedMode;
  string *pMode = mode[determinedMode];






  Serial.print(*pTeam_ID); Serial.print(",");
  Serial.print(*pMission_time); Serial.print(",");
  Serial.print(packet_count); Serial.print(",");
  Serial.print(mode); Serial.print(",");
  Serial.print(stateNames[flightState]); Serial.print(",");
  Serial.print(altitude, 2); Serial.print(",");
  Serial.print(temperature, 1); Serial.print(",");
  Serial.print(pressure_kPa, 1); Serial.print(",");
  Serial.print(battery_Voltage, 2); Serial.print(",");
  Serial.print(current, 2); Serial.print(",");
  Serial.print(gyro_Roll, 1); Serial.print(",");
  Serial.print(gyro_Pitch, 1); Serial.print(",");
  Serial.print(gyro_Yaw, 1); Serial.print(",");
  Serial.print(acceleration_roll, 2); Serial.print(",");
  Serial.print(acceleration_pitch, 2); Serial.print(",");
  Serial.print(acceleration_yaw, 2); Serial.print(",");
  Serial.print(gps_time); Serial.print(",");
  Serial.print(gps_altitude); Serial.print(",");
  Serial.print(gps_latitude, 5); Serial.print(",");
  Serial.print(gps_longitude, 5); Serial.print(",");
  Serial.print(gps_satellites); Serial.print(",");
  Serial.print(cmd_echo); Serial.print(",,");
  Serial.print(velocity); Serial.println();




}


