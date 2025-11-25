//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  LIBRARIES  ===================================
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include "teseo_liv3f_class.h"



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
float gps_lat_land;
float gps_long_land;
float future_gps_lat;
float future_gps_long;
float main_gps_vector;
float future_gps_vector;
int hh;
int mm;
int ss;
float gps_Lat;
float gps_Long;
float gps_NewLat;
float gps_NewLong;

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
unsigned long newGPSTime;
unsigned long prevGPSTime;
unsigned long gpsNewTime = 0;
int gpsTimeInterval = 5;
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
bool altitudeCommand = true;
bool manualParaglider = false;
bool eepromMode = false;
bool eepromWipe = false;
bool missionTimeBool = false;
String cmd_echo;
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
static const int ALT_BUFF = 6;
float apogeeHeight;

//                    -------- BASE PRESSURE CALIBRATION --------
unsigned long startTime;
int samplesTaken = 0;
float sum = 0;
int validCount = 0;
float tempPressurePA;
float basePressurePA;
//                    -------- AUTONOMOUS --------
int direction;
float servoOutput;
//                    -------- MECHANISMS --------
bool deployParaglider = false;
bool eggDrop = false;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  GLOBALS ===================================
//    ----- Function Declarations ----
void commandChecker(FlightData &fd);
void sendTelemetry(FlightData &fd);
void sampleData(FlightData &fd);
void UpdateFlightState(FlightData &fd);
void autonomousControls(FlightData &fd);
void altitudeCalibration(FlightData &fd);
void newGPSData(FlightData &fd);

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
// Pressure, Temp
Adafruit_BMP3XX bmp;
 // reminder to do bmp(SDA_Pin,SCL_Pin) to set it to something specific rather than default

// GPS
TeseoLIV3F *gps;
GNSSParser_Data_t data;

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

// GPS Initialization
gps = new TeseoLIV3F(&Wire,7,13);
gps->init();
Serial.println("GPS Ready");
//      --------- Timers ---------
fd.prevGPSTime = 0;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              ===================================  VOID LOOP  ===================================
void loop() {

commandChecker(fd);

altitudeCalibration(fd);

sampleData(fd);

if ((millis() - fd.gpsNewTime) > fd.gpsTimeInterval) {
  newGPSData(fd);
  fd.gpsNewTime = millis();
}


apogeeDetection(fd);

UpdateFlightState(fd);

if (fd.deployParaglider) {
  // myservo.writeMicroseconds(###);
}

if (fd.manualParaglider) {
  // myservo.writeMicroseconds(###);
}

if (fd.eggDrop) {
  // myservo.writeMicroseconds(###);
}

if (flightState = PROBE_RELEASE){
  autonomousControls(fd);
  switch(fd.direction) {
    case 1: // LEFT
      Serial.print("Left");
     // myservo.writeMicroseconds(fd.servoOutput);
      break;
    case 2: // RIGHT
      Serial.print("Right");
     // myservo.writemicroseconds(fd.servoOutput);
      break;
    case 3: // STAY STRAIGHT
      Serial.print("Stay Straight");
     // myservo.writeMicroseconds(fd.servoOutput);
      break;
  }
}


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

// GPS
// Continually makes sure data is freshhh
gps->update();
// Read that fresh data
data = gps->getData();
// Only if we have a fix, take in that data
if (data.gpgga_data.valid == 1) {
  // get latitude, is in this format: dddmm.mmmm
  float rawLat = data.gpgga_data.xyz.lat;
  int latDeg = rawLat / 100;
  float latMin = rawLat - (latDeg * 100);
  float latitude = latDeg + (latMin / 60.0);
  // to check for south
  if (data.gpgga_data.xyz.ns == 'S') {
    latitude = -latitude;
  }
  // get longitude
  float rawLon = data.gpgga_data.xyz.lon;
  int lonDeg = rawLon / 100;
  float lonMin = rawLon - (lonDeg * 100);
  float longitude = lonDeg + (lonMin / 60.0);
  // to check for west
  if (data.gpgga_data.xyz.ew == 'W') {
    longitude = -longitude;
  }
  // GPS altitude
  float gpsAltitude = data.gpgga_data.xyz.alt;
  // GPS satellites
  int gpsSats = data.gpgga_data.sats;
  // Mission Time
  int gpsHH = data.gpgga_data.utc.hh;
  int gpsMM = data.gpgga_data.utc.mm;
  int gpsSS = data.gpgga_data.utc.ss;

  // assign everything to their telemetry spot:
  fd.gps_altitude = gpsAltitude;
  fd.gps_latitude = latDeg;
  fd.gps_longitude = lonDeg;
  fd.gps_satellites = gpsSats;
  fd.hh = gpsHH;
  fd.mm = gpsMM;
  fd.ss = gpsSS;
  // assign the latitude and longitude spots for the autonomous controls
  fd.gps_Lat = latitude;
  fd.gps_Long = longitude;
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
//                                           ===================================  New GPS Data  ===================================
void newGPSData(FlightData &fd){

  // GPS for the new position
  // Continually makes sure data is freshhh
  gps->update();
  // Read that fresh data
  data = gps->getData();
  // Only if we have a fix, take in that data
  if (data.gpgga_data.valid == 1) {
  // get latitude, is in this format: dddmm.mmmm
    float rawNewLat = data.gpgga_data.xyz.lat;
    int latNewDeg = rawNewLat / 100;
    float latNewMin = rawNewLat - (latNewDeg * 100);
    float latitudeNew = latNewDeg + (latNewMin / 60.0);
  // to check for south
   if (data.gpgga_data.xyz.ns == 'S') {
      latitudeNew = -latitudeNew;
  }
  // get longitude
    float rawNewLon = data.gpgga_data.xyz.lon;
    int lonNewDeg = rawNewLon / 100;
    float lonNewMin = rawNewLon - (lonNewDeg * 100);
    float longitudeNew = lonNewDeg + (lonNewMin / 60.0);
  // to check for west
    if (data.gpgga_data.xyz.ew == 'W') {
      longitudeNew = -longitudeNew;
  }
    fd.gps_NewLat = latitudeNew;
    fd.gps_NewLong = longitudeNew;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  APOGEE DETECTION  ===================================
void apogeeDetection(FlightData &fd){
  if(!fd.apogeeDetected){
    // read new altitude
    float newAltitude = fd.altitude;
    // store in buffer
    fd.altitudeSamples[fd.altIndex] = newAltitude;
    fd.altIndex = (fd.altIndex + 1) % fd.ALT_BUFF;
    // Increment sample count until buffer is full
    if (fd.sampleCount < fd.ALT_BUFF) fd.sampleCount++;
    // only check for apogee if the buffer is full
    if (fd.sampleCount < fd.ALT_BUFF) return;
    // get recent and past readings
    float secondPast = fd.altitudeSamples[(fd.altIndex + fd.ALT_BUFF - 3) % fd.ALT_BUFF] ;
    float firstPast = fd.altitudeSamples[(fd.altIndex + fd.ALT_BUFF - 2) % fd.ALT_BUFF];
    float avgPast = (firstPast + secondPast) / (2);
    float recent = fd.altitudeSamples[(fd.altIndex + fd.ALT_BUFF - 1) % fd.ALT_BUFF];
    // check if we are descending
    if ((avgPast - recent) > 1 || ((((avgPast - recent) / avgPast) * 100) > 0.1)) {
      fd.apogeeDetected = true;
      fd.apogeeHeight = recent;
      }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  ALTITUDE CALIBRATION  ===================================
void altitudeCalibration(FlightData &fd) {
  if(fd.altitudeCommand) {
//      --------- Base Pressure Calibration ---------
  fd.startTime = millis();

  while(fd.samplesTaken < 12) {
    if ((millis() - fd.startTime) >= 80) {
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
  fd.altitudeCommand = false;
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
      fd.deployParaglider = true;
    }
    break;
  case PROBE_RELEASE:
    if (fd.altitude <= 3.5){
      flightState = PAYLOAD_RELEASE;
      fd.eggDrop = true;
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
    } else if (cmd == "CMD,1093,ST") {
      fd.missionTimeBool = true;
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
      fd.altitudeCommand = true;
    } else if (cmd == "CMD,1093,MEC,DEPLOY") {
      fd.deployParaglider = true;
    } else if (cmd == "CMD,1093,MEC,MANUAL") {
      fd.manualParaglider = true;
    } else if (cmd == "CMD,1093,MEC,EGGDROP") {
      fd.eggDrop = true;
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
//                                           ===================================  AUTONOMOUS CONTROLS  ===================================
void autonomousControls(FlightData &fd) {

  float Cx = fd.gps_Lat;
  float Cy = fd.gps_Long;
  float Nx = fd.gps_NewLat;
  float Ny = fd.gps_NewLong;
  float Fx = fd.future_gps_lat;
  float Fy = fd.future_gps_long;
  float Lx = fd.gps_lat_land;
  float Ly = fd.gps_long_land;
  float Z_direction;
  float In_min;
  float In_max;
  float Out_max;
  float Out_min;


  fd.newGPSTime = millis();
  unsigned long deltaGPSTime = (fd.newGPSTime - fd.prevGPSTime);
  // Velocity Function for Lat and Long
  float velocityGPSLat = (Nx - Cx) / deltaGPSTime;
  float velocityGPSLong = (Ny - Cy) / deltaGPSTime;
  // This is the future positon formula to eventually gain a future vector line
  Fx = Cx + (velocityGPSLat * 1000);
  Fy = Cy + (velocityGPSLong * 1000);
  // Vector Creation
  float Ax =  Fx - Cx;
  float Ay =  Fy - Cy;
  float Bx =  Lx - Cx;
  float By =  Ly - Cy;
  float A_B = (Ax * Bx) + (Ay * By);
  float A_abs = sqrt( (Ax * Ax) + (Ay * Ay) );
  float B_abs = sqrt( (Bx * Bx) + (By * By) );
  // The degree between the two Vectors formula
  float degreeVector = acos((A_B)/(A_abs * B_abs));

  // Mapping Range to Range formula to translate Angle input to Servo Output
  fd.servoOutput = Out_min + ( ( (degreeVector - In_min) * (Out_max - Out_min) ) / ( In_max - In_min  )  );


  // Used to determine which direction to Turn
  Z_direction = (Ax * By) - (Ay * Bx);
  if(Z_direction > 0.2) {
    fd.direction = 1; // Turn Left
  } else if(Z_direction < -0.2){
    fd.direction = 2; // Turn Right
  } else if(Z_direction >= -0.2 && Z_direction <= 0.2) {
    fd.direction = 3; // Z value is apporximatley 0, so we'll stay straight.
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SEND TELEMETRY  ===================================

void sendTelemetry(FlightData &fd) {

  Serial.print(fd.teamID); Serial.print(",");
  if (fd.missionTimeBool) {
    Serial.print(fd.hh); Serial.print(":");
    Serial.print(fd.mm); Serial.print(":");
    Serial.print(fd.ss); Serial.print(",");
  } else {
    Serial.print("00"); Serial.print(":");
    Serial.print("00"); Serial.print(":");
    Serial.print("00"); Serial.print(",");
  }
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
  Serial.print(fd.hh); Serial.print(":");
  Serial.print(fd.mm); Serial.print(":");
  Serial.print(fd.ss); Serial.print(",");
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////