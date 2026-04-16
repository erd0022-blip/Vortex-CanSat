//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  LIBRARIES  ===================================
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <PWMServo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TeensyThreads.h>
#include <SD.h>





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  FLIGHT DATA STRUCT  ===================================
struct FlightData {

//                    -------- GYROSCOPE --------
double gx = 67;
double gy = 67;
double gz = 67;
float Pgx = 0.0;
float Pgy = 0.0;
float Pgz = 0.0;
float Pgt = 0;
float Ngt = millis();
double ox;
double oy;
double oz;

//                    -------- ACCELERATION OF GYRO --------
double ax;
double ay;
double az;
//                    -------- GPS VARIABLES --------
double New_Lat;
double New_Long;
double GPS_Altitude;
int GPS_Hour = 0;
int GPS_Minutes = 0;
int GPS_Seconds = 0;
int GPS_Satellites;

//                    -------- MODE --------
bool determinedMode = true;
String mode;
bool cameraBool = true;
//                    -------- TIMERS --------
unsigned long missionTime;
unsigned long telemetryTime = 0;
const unsigned long telemetryInterval = 180;
unsigned long sdCardTime = 0;
int sdCardTimeInterval;
unsigned long currentTime;
float deltaTime;
unsigned long lastTime;
float PIDTimer;
float PIDInterval = 1000;
float dt;

int Thour;
int Tminute;
int Tsecond;

//                    -------- TELEMETRY --------
int packetCount = 0;


//                    -------- BATTERY --------
float battery_Voltage;
float current;
float R1;
float R2;
float ADC_REF = 5.0;
int ADC_RESOLUTION_CV = 4095;
int batteryPin = 14;
float rawSensorValue;
float VoltageInput;
float Zero_Point;
float CurrentSensorValue;
float Vpin;
float Vout;
float CurrentPin = 15;
float VoltageReading;
float Sensitivity = 0.185; //185 mV / A


//                    -------- COMMANDS --------
bool telemetryStatus = true;
float simulationPressure;
bool simulationEnable = false;
bool simulationActivate = false;
bool simulationMode = false;
bool simulationDisable = true;
bool altitudeCommand = true;
bool eepromMode = false;
bool eepromWipe = false;
bool missionTimeBool = false;
bool deployParaglider = false;
bool eggDrop = false;
String cmd_echo = "CMD_ECHO";
bool basePressureBool = true;
String cmd;

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
float prevAltitude = 0;
double deltaAltTime;
double AltTimeNow;
float rawVelocity;

//                    -------- MISC --------
float temperature; 
float pressurePA;
int teamID = 1093;

//                    -------- APOGEE DETECTION FUNCTION --------
// -------- APOGEE DETECTION FUNCTION --------
bool AltWait = false;
float secondAltitude = 0;
float thirdAltitude = 0;
float apogeeHeight = 0;
int fallingCount = 0;
float maxAltitude = 0;
bool apogeeArmed = false;

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
float ACS_Time = 0;
float ACS_Interval = 500;

double A;
double B;
double T_Lat = 34.72152;
double T_Long = -86.63714;
double C;
double D;
double C_Lat;
double C_Long;
double E;
double F;
double P_Lat;
double P_Long;
double X1;
double Y1;
double X2;
double Y2;

double Target_Bearing;
double Current_Bearing;

double TB_Degree;
double CB_Degree;
double error;

volatile int ACS_ServoDeterminer = 0;
volatile bool ServosAreAvailable = true;

float dError;
float prevError = 0;

unsigned long lastPrevUpdate = 0;
unsigned long prevInterval = 1500; // 1 second

float Proportional;
float Derivative;
float PIDOutput;
float Pgain = 1;
float Dgain = 0.4;

//                    -------- MosFets --------
int TopSMosfet = 27;
int BottomSMosfet = 39;
int RightSMosfet = 41;
int LeftSMosfet = 21;

//                    -------- SD Card --------
unsigned long SDCARDTimer = 0;
unsigned long SDCARDInterval = 250;

//                    -------- Camera MosFets --------
int TopCamera = 31;
int BottomCamera = 30;



//                    -------- Apogee --------
unsigned long AscentTimer = 0;
int ApogeeCounter = 0;
float prevApogeeAlt = 0;

//                    -------- Distance --------
double latTarget;
double longTarget;
double latCurrent;
double longCurrent;
double deltaLat;
double deltaLong;
double a;
double c;
double R = 6371;
double Distance;

//                    -------- Processor Restart --------
bool resetOn = true;


};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  GLOBALS ===================================
//    ----- Function Declarations ----
void commandChecker(FlightData &fd);
void sendTelemetry(FlightData &fd);
void sampleData(FlightData &fd);
void UpdateFlightState(void *arg);
void autonomousControls(FlightData &fd);
void altitudeCalibration(FlightData &fd);
void newGPSData(FlightData &fd);
void apogeeDetection(FlightData &fd);
void saveToSDCARD(FlightData &fd);
void ServoFunction(void *arg);

//           ---- Struct Variable ----
FlightData fd;
//      ------ STATE MACHINE FUNCTION ------
enum FlightState { 
  LAUNCH_PAD, 
  ASCENT, 
  APOGEE, 
  DESCENT, 
  PROBE_RELEASE,
  PAYLOAD_RELEASE, 
  LANDED 
  };

const char* stateNames[7] = {
  "LAUNCH_PAD", 
  "ASCENT", 
  "APOGEE", 
  "DESCENT", 
  "PROBE_RELEASE",
  "PAYLOAD_RELEASE", 
  "LANDED"
  };
 FlightState flightState = LAUNCH_PAD;

//      ------ Sensor Inits ------
// Pressure, Temp
Adafruit_BMP3XX bmp;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
// GPS
SFE_UBLOX_GNSS myGNSS;

//                    -------- SD CARD --------
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
File countState;
File resetBool;
File countFile;
// SD Reset on/off
String Bool;
String nameOfState;
int i = 0;


// Servos
PWMServo TopS;
PWMServo BottomS;
PWMServo LeftS;
PWMServo RightS;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  VOID SETUP  ===================================
void setup() {

  Serial1.begin(115200);
  Serial.begin(115200);
  SD.begin(chipSelect);
  Wire.begin();
  delay(200);

  // Setting the ADC to 12-bit
  analogReadResolution(12);

  // Reset apogee components

  //BMP 390 oversampling and filter initialization
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // GPS
  myGNSS.begin();
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(20);
  myGNSS.setAutoPVT(true);

  // SD Card
  dataFile = SD.open("Vortex_SD_CARD.txt", FILE_WRITE);

  // BNO
  bno.begin();

  //Servos
  TopS.attach(12); // J11 // Mosfet Pin: 27
  BottomS.attach(9); // J10 // Mosfet Pin: 39
  LeftS.attach(3); // J8 // Mosfet Pin: 21
  RightS.attach(6); // J9 // Mosfet Pin: 41
  delay(200);

  pinMode(fd.BottomSMosfet,OUTPUT);
  pinMode(fd.TopSMosfet,OUTPUT);
  pinMode(fd.RightSMosfet,OUTPUT);
  pinMode(fd.LeftSMosfet,OUTPUT);
  pinMode(fd.TopCamera,OUTPUT);
  pinMode(fd.BottomCamera,OUTPUT);
  digitalWrite(fd.BottomSMosfet, LOW);
  digitalWrite(fd.TopSMosfet, LOW);
  digitalWrite(fd.LeftSMosfet, LOW);
  digitalWrite(fd.RightSMosfet, LOW);
  digitalWrite(fd.TopCamera, LOW);
  digitalWrite(fd.BottomCamera, LOW);
  delay(300);

/*
  resetBool = SD.open("resetBool.txt", FILE_READ);
  if (resetBool){
    Bool = "";
    while (resetBool.available()) {
      Bool += (char)resetBool.read();
    }
    Bool.trim();
    fd.resetOn = (Bool == 1);
    resetBool.close();
  }

  countState = SD.open("flightState.txt", FILE_READ);
  if ((countState) && (fd.resetOn)) {
    nameOfState = "";
    while (countState.available()) {
      nameOfState += (char)countState.read();
    }
    nameOfState.trim();
    countState.close();
    for (int fs = 0; fs < 7; fs++) {
      if (nameOfState == stateNames[fs]) {
        i = fs;
        flightState = FlightState(i);
        break;
      }
    }

  } else {
    flightState = LAUNCH_PAD;
  }


  countFile = SD.open("PacketNum.txt", FILE_READ);
  if ((countFile) && (fd.resetOn)) {
    String savedCount = "";
    while (countFile.available()) {
      savedCount += (char)countFile.read();
    }
    savedCount.trim();
    fd.packetCount = savedCount.toInt();
    countFile.close();
  } else {
    fd.packetCount = 0;
  }

*/


  

  //Top
  digitalWrite(fd.TopSMosfet, HIGH);
  delay(500);
  TopS.write(41);
  delay(500);
  TopS.write(29);
  delay(500);
  delay(300);
  //Bottom
  digitalWrite(fd.BottomSMosfet, HIGH);
  delay(300);
  BottomS.write(180);
  delay(300);
  digitalWrite(fd.BottomSMosfet, LOW);
  delay(200);
  
  
  // Thread Function for Servo Autonomous
  threads.addThread(ServoFunction, &fd);
  threads.addThread(UpdateFlightState, &fd);


  altitudeCalibration(fd);
  fd.simulationPressure = 101325.0;



}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              ===================================  VOID LOOP  ===================================
void loop() {



  commandChecker(fd);

  altitudeCalibration(fd);

  sampleData(fd);

  newGPSData(fd);


  if(((millis()-fd.ACS_Time) > fd.ACS_Interval) && (flightState == PROBE_RELEASE)) {
    autonomousControls(fd);
    fd.ACS_Time = millis();
  }

  if(fd.telemetryStatus && ((millis() - fd.telemetryTime) >= 1000)) {
    sendTelemetry(fd);
    fd.telemetryTime = millis();
    fd.packetCount ++;
  }

  if(((millis() - fd.SDCARDTimer) >= fd.SDCARDInterval)){
    saveToSDCARD(fd);
    fd.SDCARDTimer = millis();
  }

  if ((fd.basePressureBool) && (fd.simulationMode)){
    fd.pressurePA = 101325.0;
    fd.basePressurePA = 101325.0;
    fd.basePressureBool = false;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SAMPLE DATA  ===================================
void sampleData(FlightData &fd) {

// BMP390

if(fd.simulationMode){
  fd.pressurePA = fd.simulationPressure;
} else {
  bmp.performReading();
  fd.pressurePA = bmp.pressure;
  fd.temperature = bmp.temperature;
}
/*
bmp.performReading();
fd.temperature = bmp.temperature;
*/
// Altitude
if(fd.simulationMode){
  fd.altitude = 44330.0 * (1 - pow(fd.simulationPressure / fd.basePressurePA, 0.1903));
} else {
  fd.altitude = 44330.0 * (1 - pow(fd.pressurePA / fd.basePressurePA, 0.1903));
}

//BNO
// Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

fd.gx = gyro.x();
fd.gy = gyro.y();
fd.gz = gyro.z();

float deltaAccel = (millis() - fd.Pgt) / 1000.0f;
if (deltaAccel > 0.0f) {
  fd.ax = (fd.gx - fd.Pgx) / deltaAccel;   // angular accel (rad/s^2)
  fd.ay = (fd.gy - fd.Pgy) / deltaAccel;
  fd.az = (fd.gz - fd.Pgz) / deltaAccel;
}

fd.Pgx = fd.gx;
fd.Pgy = fd.gy;
fd.Pgz = fd.gz;
fd.Pgt = millis();

// Battery
fd.rawSensorValue = analogRead(fd.batteryPin);
fd.VoltageInput = ((fd.rawSensorValue / fd.ADC_RESOLUTION_CV) * fd.ADC_REF);
fd.battery_Voltage = fd.VoltageInput * 3;

// Current Sensor
fd.Zero_Point = 0.5; //2.5 volts accoring to Pololu datasheet
fd.CurrentSensorValue = analogRead(fd.CurrentPin);
fd.Vpin = (fd.CurrentSensorValue / fd.ADC_RESOLUTION_CV) * 3.3;
fd.Vout = fd.Vpin * 2.0;
fd.current = (fd.Vout - fd.Zero_Point) / fd.Sensitivity;
fd.current = fd.current - 0.8;
/* Velocity
fd.deltaAltitude = (fd.altitude - fd.prevAltitude);
fd.deltaAltTime = (0.01);

fd.rawVelocity = (fd.deltaAltitude / fd.deltaAltTime);
if(fd.rawVelocity > 500 || fd.rawVelocity < -500) {
} else{
  fd.velocity = fd.rawVelocity;
}

if(abs(fd.altitude - fd.prevAltitude) > 0.80){
  fd.prevAltitude = fd.altitude;
}
*/


//      ----Mode Determiner Code----
if((fd.simulationMode) && (!fd.simulationDisable)) {
  fd.determinedMode = false;
} else {
  fd.determinedMode = true;
}
fd.mode = fd.determinedMode ? "F" : "S";

//      ----UTC Time----
if (fd.missionTimeBool){

  fd.Thour = fd.GPS_Hour - 5;
  fd.Tminute = fd.GPS_Minutes;
  fd.Tsecond = fd.GPS_Seconds;
}

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           =================================== GPS Data ===================================
void newGPSData(FlightData &fd){

if(myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)){
  fd.New_Lat = (myGNSS.getLatitude() / (pow(10,7)));
  fd.New_Long = (myGNSS.getLongitude() / (pow(10,7)));
  fd.GPS_Altitude = myGNSS.getAltitude();
  fd.GPS_Hour = myGNSS.getHour();
  fd.GPS_Minutes = myGNSS.getMinute();
  fd.GPS_Seconds = myGNSS.getSecond();
  fd.GPS_Satellites = myGNSS.getSIV();

  fd.C_Lat = fd.New_Lat;
  fd.C_Long = fd.New_Long;

  if(millis() - fd.lastPrevUpdate >= fd.prevInterval ) {
    fd.P_Lat = fd.C_Lat;
    fd.P_Long = fd.C_Long;
    fd.lastPrevUpdate = millis();
    }

  // Distance using Haversine Formula
  fd.latTarget = fd.T_Lat * (PI / 180);
  fd.longTarget = fd.T_Long * (PI / 180);
  fd.latCurrent = fd.New_Lat * (PI / 180);
  fd.longCurrent = fd.New_Long * (PI / 180);
  fd.deltaLat = fd.latTarget - fd.latCurrent;
  fd.deltaLong = fd.longTarget - fd.longCurrent;

  fd.a = sin(fd.deltaLat / 2)*(sin(fd.deltaLat / 2)) + cos(fd.latCurrent)*cos(fd.latTarget)*(sin(fd.deltaLong / 2)*(sin(fd.deltaLong / 2)));
  fd.c = 2 * atan2(sqrt(fd.a),sqrt(1-fd.a));
  fd.Distance = fd.R * fd.c;
  

  }


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  APOGEE DETECTION  ===================================
void saveToSDCARD(FlightData &fd) {

  dataFile = SD.open("Vortex_SD_CARD.txt", FILE_WRITE);
  dataFile.print(fd.teamID); dataFile.print(",");
  dataFile.print(fd.Thour, 2); dataFile.print(":");
  dataFile.print(fd.Tminute, 2); dataFile.print(":");
  dataFile.print(fd.Tsecond, 2); dataFile.print(",");
  dataFile.print(fd.packetCount); dataFile.print(",");
  dataFile.print(fd.mode); dataFile.print(",");
  dataFile.print(stateNames[flightState]); dataFile.print(",");
  dataFile.print(fd.altitude, 2); dataFile.print(",");
  dataFile.print(fd.temperature, 1); dataFile.print(",");
  dataFile.print(fd.pressurePA, 1); dataFile.print(",");
  dataFile.print(fd.battery_Voltage, 2); dataFile.print(",");
  dataFile.print(fd.current, 3 ); dataFile.print(",");
  dataFile.print(fd.gx, 1); dataFile.print(",");
  dataFile.print(fd.gy, 1); dataFile.print(",");
  dataFile.print(fd.gz, 1); dataFile.print(",");
  dataFile.print(fd.ax, 2); dataFile.print(",");
  dataFile.print(fd.ay, 2); dataFile.print(",");
  dataFile.print(fd.az, 2); dataFile.print(",");
  dataFile.print(fd.GPS_Hour); dataFile.print(":");
  dataFile.print(fd.GPS_Minutes); dataFile.print(":");
  dataFile.print(fd.GPS_Seconds); dataFile.print(",");
  dataFile.print(fd.GPS_Altitude); dataFile.print(",");
  dataFile.print(fd.New_Lat, 5); dataFile.print(",");
  dataFile.print(fd.New_Long , 5); dataFile.print(",");
  dataFile.print(fd.GPS_Satellites); dataFile.print(",");
  dataFile.print(fd.apogeeHeight); dataFile.print(",");
  dataFile.print(fd.basePressurePA); dataFile.print(",");
  dataFile.println(fd.cmd_echo);
  dataFile.flush();
  dataFile.close();


  // Processor Restart
  countState = SD.open("flightStateTest.txt", FILE_WRITE);
  if (countState) {
    countState.seek(0);
    countState.print(stateNames[flightState]);
    countState.truncate(String(stateNames[flightState]).length());
    countState.close();
  }

  resetBool = SD.open("resetBool.txt", FILE_WRITE);
  if (resetBool) {
    resetBool.seek(0);
    resetBool.print(fd.resetOn);
    resetBool.close();
  }

  countFile = SD.open("PacketNum.txt", FILE_WRITE);
  if (countFile) {
    countFile.seek(0);
    countFile.print(fd.packetCount);
    countFile.truncate(String(fd.packetCount).length());
    countFile.close();
  }




}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  ALTITUDE CALIBRATION  ===================================
void altitudeCalibration(FlightData &fd) {
  if(fd.altitudeCommand) {
//      --------- Base Pressure Calibration ---------
  fd.startTime = millis();

  while(fd.samplesTaken < 6) {
    if ((millis() - fd.startTime) >= 80) {
      if(fd.simulationMode){
        fd.tempPressurePA = fd.simulationPressure;
      } else {
        bmp.performReading();
        fd.tempPressurePA = bmp.pressure;
      }

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
void UpdateFlightState(void *arg) {
  FlightData &fd = *(FlightData*)arg;


while(1){
    switch(flightState) {
    case LAUNCH_PAD:
      if (fd.altitude > 20.00 ) {
        flightState = ASCENT;
        digitalWrite(fd.TopCamera, HIGH);
        digitalWrite(fd.BottomCamera, HIGH);
      }
      break;
    case ASCENT:
      if (fd.altitude < fd.prevApogeeAlt + 0.2){
        fd.ApogeeCounter++;
        if (fd.ApogeeCounter == 1) {
          fd.apogeeHeight = fd.altitude;
        } else if(fd.ApogeeCounter >= 10){
          flightState = APOGEE;
        }
      } else {
        fd.ApogeeCounter = 0;
      }
      fd.prevApogeeAlt = fd.altitude;
      break;
    case APOGEE:
      if (fd.altitude < fd.apogeeHeight){
        flightState = DESCENT;
      }
      break;
    case DESCENT:
      if (fd.altitude <= (fd.apogeeHeight * 0.80)){
        flightState = PROBE_RELEASE;
        digitalWrite(fd.TopSMosfet, HIGH);
        threads.delay(300);
        TopS.write(41);
        threads.delay(300);
        digitalWrite(fd.TopSMosfet, LOW);
      }
      break;
    case PROBE_RELEASE:

        digitalWrite(fd.RightSMosfet, HIGH);
        digitalWrite(fd.LeftSMosfet, HIGH);

      if (fd.altitude <= 11){
        flightState = PAYLOAD_RELEASE;
        digitalWrite(fd.BottomSMosfet, HIGH);
        delay(300);
        BottomS.write(0);
        threads.delay(600);
        digitalWrite(fd.BottomSMosfet, LOW);
        threads.delay(200);
      }
      break;
    case PAYLOAD_RELEASE:

      if ((fd.altitude > -5) && (fd.altitude < 5)){
        flightState = LANDED;


      }
      break;
    case LANDED:

      break;
    }

  threads.yield();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  Command Checker Function  ===================================

void commandChecker(FlightData &fd){

  bool gotCommand = false;

  if (Serial1.available()) {
    fd.cmd = Serial1.readStringUntil('\n');
    fd.cmd.trim();
    fd.cmd_echo = fd.cmd;
    gotCommand = true;

  } else if (Serial.available()) {
    fd.cmd = Serial.readStringUntil('\n');
    fd.cmd.trim();
    fd.cmd_echo = fd.cmd;
    gotCommand = true;

  }

  if(!gotCommand){
    return;
  }

    if (fd.cmd == "CMD,1093,CX,ON") {
      fd.telemetryStatus = true;
    } else if (fd.cmd == "CMD,1093,CX,OFF") {
      fd.telemetryStatus = false;

    } else if (fd.cmd == "CMD,1093,ST") {
      fd.missionTimeBool = true;

    } else if (fd.cmd == "CMD,1093,SIM,ENABLE") {
      fd.simulationEnable = true;
      fd.simulationDisable = false;
      
    } else if (fd.cmd == "CMD,1093,SIM,ACTIVATE") {
      fd.simulationActivate = true;
      fd.simulationPressure = 101325.0;
      fd.basePressurePA = 101325.0;

    } else if (fd.cmd == "CMD,1093,SIM,DISABLE") {
      fd.simulationDisable = true;
      fd.simulationActivate = false;
      fd.simulationEnable = false;
      fd.simulationMode = false;
      fd.basePressureBool = true;

    } else if (fd.cmd == "CMD,1093,CAL") {
      fd.altitudeCommand = true;
      fd.samplesTaken = 0;
      fd.sum = 0;
      fd.validCount = 0;

    } else if (fd.cmd == "CMD,1093,SD,ON"){
      fd.resetOn = true;

    } else if (fd.cmd == "CMD,1093,SD,OFF") {
      fd.resetOn = false;

  
  
    }else if (fd.cmd == "CMD,1093,MEC,PROBE,UNLOCK") {
      digitalWrite(fd.TopSMosfet, HIGH);
      delay(300);
      TopS.write(41);
      delay(300);
      digitalWrite(fd.TopSMosfet, LOW);

    } else if (fd.cmd == "CMD,1093,MEC,PROBE,LOCK") {
      digitalWrite(fd.TopSMosfet, HIGH);
      delay(300);
      TopS.write(29);
      delay(2000);
      digitalWrite(fd.TopSMosfet, LOW);

    } else if (fd.cmd == "CMD,1093,MEC,EGG,UNLOCK") {
      digitalWrite(fd.BottomSMosfet, HIGH);
      delay(300);
      BottomS.write(0);
      delay(300);
      digitalWrite(fd.BottomSMosfet, LOW);
    
    } else if (fd.cmd == "CMD,1093,MEC,EGG,LOCK") {
      digitalWrite(fd.BottomSMosfet, HIGH);
      delay(300);
      BottomS.write(180);
      delay(300);
      digitalWrite(fd.BottomSMosfet, LOW);

    } else if (fd.cmd == "CMD,1093,MEC,ACS,LEFT") {
      digitalWrite(fd.LeftSMosfet, HIGH);
      delay(300);
      fd.ServosAreAvailable = false;
      LeftS.write(180);
      delay(1000);
      LeftS.write(0);
      delay(1005);
      LeftS.write(90);
      delay(300);
      digitalWrite(fd.LeftSMosfet, LOW);
      fd.ServosAreAvailable = true;

    } else if (fd.cmd == "CMD,1093,MEC,ACS,RIGHT") {
      digitalWrite(fd.RightSMosfet, HIGH);
      delay(300);
      fd.ServosAreAvailable = false;
      RightS.write(0);
      delay(1000);
      RightS.write(180);
      delay(1031);
      RightS.write(90);
      delay(300);
      digitalWrite(fd.RightSMosfet, LOW);
      fd.ServosAreAvailable = true;

    }

     fd.simulationMode = fd.simulationEnable && fd.simulationActivate && !fd.simulationDisable;

    if(fd.cmd.startsWith("CMD,1093,SIMP,")){
      if (fd.simulationMode){
        fd.simulationPressure = fd.cmd.substring(fd.cmd.lastIndexOf(",") + 1).toFloat();
      }
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SERVO FUNCTION THREAD  ===================================
void ServoFunction(void *arg) {
  FlightData &fd = *(FlightData*)arg;

while(1){
  if(fd.ACS_ServoDeterminer == 4 && fd.ServosAreAvailable) {

    fd.ServosAreAvailable = false;
    RightS.write(0);
    threads.delay(1000);
    RightS.write(180);
    threads.delay(1031);
    RightS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0;

  }
  if(fd.ACS_ServoDeterminer == 5 && fd.ServosAreAvailable) {

    fd.ServosAreAvailable = false;
    RightS.write(0);
    threads.delay(2000);
    RightS.write(180);
    threads.delay(2062);
    RightS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0; 

  }
  /*if(fd.ACS_ServoDeterminer == 6 && fd.ServosAreAvailable) {
    fd.ServosAreAvailable = false;
    RightS.write(0);
    threads.delay(3000);
    RightS.write(180);
    threads.delay(3093);
    RightS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0;
  }*/
  if(fd.ACS_ServoDeterminer == 1  && fd.ServosAreAvailable) {

    fd.ServosAreAvailable = false;
    LeftS.write(180);
    threads.delay(1000);
    LeftS.write(0);
    threads.delay(1005);
    LeftS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0;

  }
  if(fd.ACS_ServoDeterminer == 2 && fd.ServosAreAvailable) {

    fd.ServosAreAvailable = false;
    LeftS.write(180);
    threads.delay(2000);
    LeftS.write(0);
    threads.delay(2010);
    LeftS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0;

  }
  /*if(fd.ACS_ServoDeterminer == 3 && fd.ServosAreAvailable) {
    fd.ServosAreAvailable = false;
    LeftS.write(180);
    threads.delay(3000);
    LeftS.write(0);
    threads.delay(3015);
    LeftS.write(90);
    fd.ServosAreAvailable = true;
    fd.ACS_ServoDeterminer = 0;
  }*/

  if(flightState == LANDED){
      threads.delay(7000);
      digitalWrite(fd.TopCamera, LOW);
      digitalWrite(fd.BottomCamera, LOW);
  }

  threads.yield();
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  AUTONOMOUS CONTROLS  ===================================
void autonomousControls(FlightData &fd) {

//Begin Assigning GPS variables

// Target GPS Cords
fd.A = fd.T_Lat;
fd.B = fd.T_Long;

// Current GPS Cords
fd.C = fd.C_Lat;
fd.D = fd.C_Long;

// Previous GPS Cords
fd.E = fd.P_Lat;
fd.F = fd.P_Long;

// Make my atan2 values
fd.X1 = (fd.A - fd.C);
fd.Y1 = (fd.B - fd.D)*(cos((fd.A + fd.C)/2));
fd.X2 = (fd.C - fd.E);
fd.Y2 = (fd.D - fd.F)*(cos((fd.D + fd.F)/2));

fd.Target_Bearing = atan2(fd.Y1,fd.X1);
fd.Current_Bearing = atan2(fd.Y2, fd.X2);



// Change the outputs to degrees
fd.TB_Degree = fd.Target_Bearing * (180 / PI);
fd.CB_Degree = fd.Current_Bearing * (180 / PI);

// Wrap the degree to be from 0-360. 0 (North), 90 (East), 180 (South), 270 (West)
if (fd.TB_Degree < 0) {
  fd.TB_Degree += 360;
}

if (fd.CB_Degree < 0) {
  fd.CB_Degree += 360;
}

// Both of these should be somewhere between 0 and 360. If we subtract this is our error.

fd.error = (fd.TB_Degree - fd.CB_Degree);
// Wrap the error from -180 to 180 to take the quickest route when it comes to fixing our direction
if (fd.error > 180){
  fd.error -= 360;
} else if (fd.error <= -180){
  fd.error += 360;
}

/*if (fd.error > 0){
  Serial.print("Use Left Servo");
}
if (fd.error < 0){
  Serial.print("Use Right Servo");
}
*/
if((millis() - fd.PIDTimer) > fd.PIDInterval){
  fd.dt = (millis() - fd.PIDTimer) / 1000.f;
  fd.PIDTimer = millis();

  fd.dError = fd.error - fd.prevError;
  fd.prevError = fd.error;

  fd.Proportional = fd.error;
  fd.Derivative = fd.dError / fd.dt;
  fd.PIDOutput = (fd.Proportional * fd.Pgain) + (fd.Derivative * fd.Dgain);
}


if(fd.error > 0){
  if(fd.PIDOutput > 0 && fd.PIDOutput <= 90) {
    fd.ACS_ServoDeterminer = 1;
  } else if(fd.PIDOutput > 90 && fd.PIDOutput <= 180) {
    fd.ACS_ServoDeterminer = 2;
 // } else if(fd.PIDOutput > 120 && fd.PIDOutput <= 180) {
 //   fd.ACS_ServoDeterminer = 3;
  } else if(fd.PIDOutput < 0 && fd.PIDOutput >= -90) {
    fd.ACS_ServoDeterminer = 4;
  } else if(fd.PIDOutput < -90 && fd.PIDOutput >= -180 ) {
    fd.ACS_ServoDeterminer = 5;
  //} else if(fd.PIDOutput < -120 && fd.PIDOutput >= -180 ) {
  //  fd.ACS_ServoDeterminer = 6;
  }
}
if(fd.error < 0){
    if(fd.PIDOutput > 0 && fd.PIDOutput <= 90) {
    fd.ACS_ServoDeterminer = 4;
  } else if(fd.PIDOutput > 90 && fd.PIDOutput <= 180) {
    fd.ACS_ServoDeterminer = 5;
 // } else if(fd.PIDOutput > 120 && fd.PIDOutput <= 180) {
  //  fd.ACS_ServoDeterminer = 6;
  } else if(fd.PIDOutput < 0 && fd.PIDOutput >= -90) {
    fd.ACS_ServoDeterminer = 1;
  } else if(fd.PIDOutput < -90 && fd.PIDOutput >= -180 ) {
    fd.ACS_ServoDeterminer = 2;
 // } else if(fd.PIDOutput < -120 && fd.PIDOutput >= -180 ) {
 //   fd.ACS_ServoDeterminer = 3;
  }
}











}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ===================================  SEND TELEMETRY  ===================================

void sendTelemetry(FlightData &fd) {

  Serial1.print(fd.teamID); Serial1.print(",");
  Serial1.print(fd.Thour); Serial1.print(":");
  Serial1.print(fd.Tminute); Serial1.print(":");
  Serial1.print(fd.Tsecond); Serial1.print(",");
  Serial1.print(fd.packetCount); Serial1.print(",");
  Serial1.print(fd.mode); Serial1.print(",");
  Serial1.print(stateNames[flightState]); Serial1.print(",");
  Serial1.print(fd.altitude, 2); Serial1.print(",");
  Serial1.print(fd.temperature, 1); Serial1.print(",");
  if(!fd.simulationMode){
  Serial1.print(fd.pressurePA, 1); Serial1.print(",");
  } else {
  Serial1.print(fd.simulationPressure, 1); Serial1.print(",");
  }
  Serial1.print(fd.battery_Voltage, 2); Serial1.print(",");
  Serial1.print(fd.current, 2); Serial1.print(",");
  Serial1.print(fd.gx, 1); Serial1.print(",");
  Serial1.print(fd.gy, 1); Serial1.print(",");
  Serial1.print(fd.gz, 1); Serial1.print(",");
  Serial1.print(fd.ax, 2); Serial1.print(",");
  Serial1.print(fd.ay, 2); Serial1.print(",");
  Serial1.print(fd.az, 2); Serial1.print(",");
  Serial1.print(fd.GPS_Hour); Serial1.print(":");
  Serial1.print(fd.GPS_Minutes); Serial1.print(":");
  Serial1.print(fd.GPS_Seconds); Serial1.print(",");
  Serial1.print(fd.GPS_Altitude); Serial1.print(",");
  Serial1.print(fd.New_Lat, 5); Serial1.print(",");
  Serial1.print(fd.New_Long , 5); Serial1.print(",");
  Serial1.print(fd.GPS_Satellites); Serial1.print(",");
  Serial1.println(fd.cmd_echo); 
  
  //Serial.print(",,");
 /* if(!fd.simulationDisable) {
    Serial.print(fd.simulationPressure); Serial.print(",");
  } else {
    fd.simulationPressure = 0;
    Serial.print(fd.simulationPressure); Serial.print(",");
  }
  */
  //Serial.print(fd.velocity); Serial.println();

  Serial.print(fd.teamID); Serial.print(",");
  Serial.print(fd.Thour); Serial.print(":");
  Serial.print(fd.Tminute); Serial.print(":");
  Serial.print(fd.Tsecond); Serial.print(",");
  Serial.print(fd.packetCount); Serial.print(",");
  Serial.print(fd.mode); Serial.print(",");
  Serial.print(stateNames[flightState]); Serial.print(",");
  Serial.print(fd.altitude, 2); Serial.print(",");
  Serial.print(fd.temperature, 1); Serial.print(",");
  Serial.print(fd.pressurePA, 1); Serial.print(",");
  Serial.print(fd.battery_Voltage, 2); Serial.print(",");
  Serial.print(fd.current, 2); Serial.print(",");
  Serial.print(fd.gx, 1); Serial.print(",");
  Serial.print(fd.gy, 1); Serial.print(",");
  Serial.print(fd.gz, 1); Serial.print(",");
  Serial.print(fd.ax, 2); Serial.print(",");
  Serial.print(fd.ay, 2); Serial.print(",");
  Serial.print(fd.az, 2); Serial.print(",");
  Serial.print(fd.GPS_Hour); Serial.print(":");
  Serial.print(fd.GPS_Minutes); Serial.print(":");
  Serial.print(fd.GPS_Seconds); Serial.print(",");
  Serial.print(fd.GPS_Altitude); Serial.print(",");
  Serial.print(fd.New_Lat, 5); Serial.print(",");
  Serial.print(fd.New_Long , 5); Serial.print(",");
  Serial.print(fd.GPS_Satellites); Serial.print(",");
  Serial.print(fd.velocity); Serial.print(",");
  Serial.println(fd.cmd_echo);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////