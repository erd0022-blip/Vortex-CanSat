//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  LIBRARIES  ===================================
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  FLIGHT DATA STRUCT  ===================================
struct FlightData {

//                    -------- GYROSCOPE --------
float gyro_Roll;
float gyro_Pitch;
float gyro_Yaw;
//                    -------- ACCELERATION OF GYRO --------
float acceleration_roll;
float acceleration_pitch;
float acceleration_yaw;
//                    -------- GPS VARIABLES --------
float gps_time;
float gps_altitude;
float gps_latitude;
float gps_longitude; 
int gps_satellites;
//                    -------- MODE --------
bool determinedMode;
String mode;
//                    -------- TIMERS --------
unsigned long missionTime;
unsigned long telemetryTime = 0;
int telemetryInterval = 1000;
unsigned long sdCardTime = 0;
int sdCardTimeInterval;
unsigned long currentTime;
float deltaTime;
unsigned long lastTime;
//                    -------- TELEMETRY --------
int packetCount;
//                    -------- BATTERY --------
float battery_Voltage;
float current;
//                    -------- COMMANDS --------
bool telemetryStatus = false;
float simulationPressure;
bool simulationEnable = false;
bool simulationActivate = false;
bool simulationDisable = true;
bool calibrateAltitude = false;
bool deployParaglider = false;
bool manualParaglider = false;
bool eepromMode = false;
bool eepromWipe = false;
String cmd_echo;
//                    -------- MECHANISMS --------
bool paragliderRelease = false;
bool dropEgg = false;
//                    -------- Altitude --------
float altitude;
float altitudeRaw;
float altitudeFiltered;
float altitudeFilt;
float alpha = 0.06;
float deltaAltitude;
float lastAltitude;
//                    -------- Velocity --------
float velocity;
//                    -------- MISC --------
float temperature; 
float pressure_Pa;
int teamID = 1093;

//                    -------- APOGEE DETECTION FUNCTION --------
bool apogeeDetected = false;
int sampleCount = 0;
int altIndex;
float altitudeSamples[6];
static const int BUFFER_SIZE = 6;
float apogeeHeight;

//                    -------- BASE pRESSURE CALIBRATION --------
unsigned long startTime;
int samplesTaken = 0;
float sum = 0;
int validCount = 0;
float tempPressurePA;
float basePressurePA;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  GLOBALS ===================================
//    ----- Function Declarations ----
void commandChecker(FlightData &fd);
void sendTelemetry(FlightData &fd);
void sampleData(FlightData &fd);
void UpdateFlightState(FlightData &fd);

//           ---- Struct Variable ----
FlightData fd;
//      ------ STATE MACHINE FUNCTION ------
enum FlightState { 
  LAUNCHPAD, 
  ASCENT, 
  APOGEE, 
  DESCENT, 
  PROBE_RELEASE,
  PAYLOAD_RELEASE, 
  LANDED 
  };

const char* stateNames[7] = {
  "LAUNCHPAD", 
  "ASCENT", 
  "APOGEE", 
  "DESCENT", 
  "PROBE_RELEASE",
  "PAYLOAD_RELEASE", 
  "LANDED"
  };
 FlightState flightState = LAUNCHPAD;

//      ------ Sensor Inits ------
Adafruit_BMP3XX bmp;
 // you can always do bmp(SDA_Pin,SCL_Pin) to set it to something specific rather than default
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  VOID SETUP  ===================================
void setup() {
Serial.begin(115200);
Wire.begin();


// Set up oversampling and filter initialization
bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
bmp.setOutputDataRate(BMP3_ODR_50_HZ);

if (!bmp.begin_I2C()) {
    Serial.println("BMP390 Not Initialized");
}

//      --------- Base Pressure Calibration ---------
  fd.startTime = millis();

  while(fd.samplesTaken < 12) {
    if ((millis() - fd.startTime) >= 250) {
      fd.tempPressurePA = bmp.pressure;
      if (fd.tempPressurePA >= 95000.0 && fd.tempPressurePA <= 105000.0 ) {
        fd.sum += fd.tempPressurePA;
        fd.validCount++;
      }
      fd.samplesTaken++;
      fd.startTime = millis();
    }
  }

  fd.basePressurePA = (fd.validCount > 0) ? fd.sum / fd.validCount : 101325.0;


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              ===================================  VOID LOOP  ===================================
void loop() {

commandChecker(fd);

sampleData(fd);

apogeeDetection(fd);

UpdateFlightState(fd);

if (fd.telemetryStatus){
  if((millis() - fd.telemetryTime) > fd.telemetryInterval){
    sendTelemetry(fd);
    fd.telemetryTime = millis();
    }
  }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SAMPLE DATA  ===================================
void sampleData(FlightData &fd) {
bmp.performReading();
//      ----BMP390 Sampling Code----
// Pressure
fd.pressure_Pa = bmp.pressure;
// Temperature
fd.temperature = bmp.temperature;

// Altitude
fd.altitudeRaw = 44330 * (1 - pow((fd.pressure_Pa / fd.basePressurePA), 1.0 / 5.255));
fd.altitudeFiltered = fd.altitudeRaw;
fd.altitudeFilt = fd.alpha * fd.altitudeRaw + (1 - fd.alpha) * fd.altitudeFiltered;
// Use Filtered Altitude
fd.altitude = fd.altitudeFilt;

// Velocity
fd.currentTime = millis();
fd.deltaAltitude = fd.altitude - fd.lastAltitude;
fd.deltaTime = (fd.currentTime - fd.lastTime) / 1000.0;
if (fd.deltaTime > 0.02) {
  fd.velocity = fd.deltaAltitude / fd.deltaTime;
  if (fabs(fd.velocity) < 0.4) {
    fd.velocity = 0.0;
  }
} else {
  return;
} 

//      ----Mode Determiner Code----
if(fd.simulationEnable && fd.simulationActivate) {
  fd.determinedMode = false;
} else {
  fd.determinedMode = true;
}
fd.mode = fd.determinedMode ? "F" : "S";


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  APOGEE DETECTION  ===================================
void apogeeDetection(FlightData &fd){
  if(!fd.apogeeDetected){
    // read new altitude
    float newAltitude = fd.altitude;
    // store in buffer
    fd.altitudeSamples[fd.altIndex] = newAltitude;
    fd.altIndex = (fd.altIndex + 1) % fd.BUFFER_SIZE;
    // Increment sample count until buffer is full
    if (fd.sampleCount < fd.BUFFER_SIZE) fd.sampleCount++;
    // only check for apogee if the buffer is full
    if (fd.sampleCount < fd.BUFFER_SIZE) return;
    // get recent and past readings
    float secondPast = fd.altitudeSamples[(fd.altIndex + fd.BUFFER_SIZE - 3) % fd.BUFFER_SIZE] ;
    float firstPast = fd.altitudeSamples[(fd.altIndex + fd.BUFFER_SIZE - 2) % fd.BUFFER_SIZE];
    float avgPast = (firstPast + secondPast) / (2);
    float recent = fd.altitudeSamples[(fd.altIndex + fd.BUFFER_SIZE - 1) % fd.BUFFER_SIZE];
    // check if we are descending
    if ((avgPast - recent) > 1 || ((((avgPast - recent) / avgPast) * 100) > 0.1)) {
      fd.apogeeDetected = true;
      fd.apogeeHeight = recent;
      }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  FLIGHT STATE  ===================================
void UpdateFlightState(FlightData &fd) {
  switch(flightState) {
  case LAUNCHPAD:
    if (fd.altitude > 10 ) {
      flightState = ASCENT;
    }
    break;
  case ASCENT: 
      if (fd.apogeeDetected) {
        flightState = APOGEE;
        Serial1.print(fd.apogeeHeight);
    } else if (fd.velocity > -0.5 && fd.velocity < 0.5 && fd.altitude > 500) {
        flightState = APOGEE;
      } 
    break;
  case APOGEE:
    if (fd.altitude < (fd.apogeeHeight + 0.2)){
      flightState = DESCENT;
    }
    break;
  case DESCENT:
    if (fd.altitude <= (fd.apogeeHeight * 0.80)){
      flightState = PROBE_RELEASE;
      fd.paragliderRelease = true;
    }
    break;
  case PROBE_RELEASE:
    if (fd.altitude <= 3.5){
      flightState = PAYLOAD_RELEASE;
      fd.dropEgg = true;
    }
    break;
  case PAYLOAD_RELEASE:
    if ((fd.altitude > -1 && fd.altitude < 1) || (fd.velocity > -1 && fd.velocity < 1)){
      flightState = LANDED;
    }
    break;
  case LANDED:
    break;
  }
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  Command Checker Function  ===================================

void commandChecker(FlightData &fd){

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    fd.cmd_echo = cmd;

    if (cmd == "CMD,1093,CX,ON") {
      fd.telemetryStatus = true;
    } else if (cmd == "CMD,1093,CX,OFF") {
      fd.telemetryStatus = false;
    } else if (cmd == "CMD,1093,SIM,ENABLE") {
      fd.simulationEnable = true;
      fd.simulationDisable = false;
    } else if (cmd == "CMD,1093,SIM,ACTIVATE" && fd.simulationEnable) {
      fd.simulationActivate = true;
    } else if (cmd == "CMD,1093,SIM,DISABLE") {
      fd.simulationDisable = true;
      fd.simulationActivate = false;
      fd.simulationEnable = false;
    } else if (cmd == "CMD,1093,CAL") {
      fd.calibrateAltitude = true;
    } else if (cmd == "CMD,1093,MEC,DEPLOY") {
      fd.deployParaglider = true;
    } else if (cmd == "CMD,1093,MEC,MANUAL") {
      fd.manualParaglider = true;
    } else if (cmd == "CMD,1093,EEPROM,ENABLE") {
      fd.eepromMode = true;
    } else if (cmd == "CMD,1093,EEPROM,DISABLE") {
      fd.eepromMode = false;
    } else if (cmd == "CMD,1093,EEPROM,WIPE") {
      fd.eepromWipe = true;
    } else if (fd.simulationEnable && fd.simulationActivate) {
      if (cmd.startsWith("CMD,1093,SIMP")) {
        int lastComma = cmd.lastIndexOf(",");
        if (lastComma != -1) {
          String pressure_Pa = cmd.substring(lastComma + 1);
          fd.simulationPressure = pressure_Pa.toFloat();
        }
      }
    }  
  }




}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SEND TELEMETRY  ===================================

void sendTelemetry(FlightData &fd) {

  Serial.print(fd.teamID); Serial.print(",");
  Serial.print(fd.missionTime); Serial.print(",");
  Serial.print(fd.packetCount); Serial.print(",");
  Serial.print(fd.mode); Serial.print(",");
  Serial.print(stateNames[flightState]); Serial.print(",");
  Serial.print(fd.altitude, 2); Serial.print(",");
  Serial.print(fd.temperature, 1); Serial.print(",");
  Serial.print(fd.pressure_Pa / 1000, 1); Serial.print(",");
  Serial.print(fd.battery_Voltage, 2); Serial.print(",");
  Serial.print(fd.current, 2); Serial.print(",");
  Serial.print(fd.gyro_Roll, 1); Serial.print(",");
  Serial.print(fd.gyro_Pitch, 1); Serial.print(",");
  Serial.print(fd.gyro_Yaw, 1); Serial.print(",");
  Serial.print(fd.acceleration_roll, 2); Serial.print(",");
  Serial.print(fd.acceleration_pitch, 2); Serial.print(",");
  Serial.print(fd.acceleration_yaw, 2); Serial.print(",");
  Serial.print(fd.gps_time); Serial.print(",");
  Serial.print(fd.gps_altitude); Serial.print(",");
  Serial.print(fd.gps_latitude, 5); Serial.print(",");
  Serial.print(fd.gps_longitude, 5); Serial.print(",");
  Serial.print(fd.gps_satellites); Serial.print(",");
  Serial.print(fd.cmd_echo); Serial.print(",,");
  if(!fd.simulationDisable) {
    Serial.print(fd.simulationPressure); Serial.print(",");
  } else {
    fd.simulationPressure = 0;
    Serial.print(fd.simulationPressure); Serial.print(",");
  }
  Serial.print(fd.velocity); Serial.println();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
