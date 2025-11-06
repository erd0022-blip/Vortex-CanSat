
//                                     ===================================  FUNCTION DECLARATIONS ===================================
void sendTelemetry();
void sampleData();
void UpdateFlightState();

enum FlightState { 
  LAUNCHPAD, 
  ASCENT, 
  APOGEE, 
  DESCENT, 
  PAYLOAD_RELEASED, 
  PROBE_RELEASED,
  LANDED 
  };

FlightState flightState = LAUNCHPAD;
const char* stateNames[] = {
  "LAUNCHPAD", 
  "ASCENT", 
  "APOGEE", 
  "DESCENT", 
  "PAYLOAD_RELEASED", 
  "PROBE_RELEASED",
  "LANDED"
  };

//                                           ===================================  TIMERS  ===================================

//                                           ===================================  VOID SETUP  ===================================
void setup() {
Serial.begin(115200);



}
//                                           ===================================  VOID LOOP  ===================================
void loop() {


}

//                                           ===================================  SAMPLE DATA  ===================================
void sampleData(unsigned long& missionTime, int& packetCount, int& determinedMode, String& mode, 
                float& altitude, float& temperature, float& pressure_kpa, float& battery_voltage, float& current, float& gyro_Roll, float& gyro_Pitch,
                float& gyro_Yaw, float& acceleration_roll, float& acceleration_pitch, float& acceleration_yaw, float& gps_time, float& gps_altitude, float& gps_latitude, float& gps_longitude, 
                int& gps_satellites, String& cmd_echo, float& velocity) {





missionTime;
packetCount;
mode = determinedMode ? "F" : "S";




}

//                                           ===================================  FLIGHT STATE  ===================================
void UpdateFlightState(FlightState& flightState) {
  switch(flightState) {
    //LaunchPad
  case LAUNCHPAD
    if (acceleration > 20 ) {
      flightState = ASCENT
    }
    break;
  case ASCENT 
      if (apogeeDetected) {
        flightState = APOGEE;
    } else if (velocity > -0.5 && velocity < 0.5 && altitude > 1400) {
        flightState = APOGEE;
      } 
    break;
  case APOGEE
    if ()

  }
}

//                                           ===================================  SEND TELEMETRY  ===================================

void sendTelemetry(unsigned long missionTime, int packetCount, int determinedMode, String mode, 
                   FlightState flightState, float altitude, float temperature, float pressure_kpa, float battery_voltage, float current, float gyro_Roll, float gyro_Pitch,
                   float gyro_Yaw, float acceleration_roll, float acceleration_pitch, float acceleration_yaw, float gps_time, float gps_altitude, float gps_latitude, float gps_longitude, 
                   int gps_satellites, String cmd_echo, float velocity) {

int teamID = 1093;

  Serial.print(teamID); Serial.print(",");
  Serial.print(missionTime); Serial.print(",");
  Serial.print(packetCount); Serial.print(",");
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


