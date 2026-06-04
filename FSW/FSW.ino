//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |          ===================== Libraries ======================           |
//                                                        |///////////////////////////////////////////////////////////////////////////|
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

//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |       ===================== Struct Variables ======================       |
//                                                        |///////////////////////////////////////////////////////////////////////////|
struct FlightData {

//      |========================|
//      |COMMAND CHECKER Section |
//      |========================|
// * Toggles telemetry (on / off)
bool telemetryStatus = false;
// * (1 of 2) requirements to engage simulation mode
bool simulationEnable = false;
// * (2 of 2) requirements to engage simulation mode
bool simulationActivate = false;
// * Used to determine the mode (flight / simulation)
bool simulationMode = false;
// * To disengage from simulation mode
bool simulationDisable = true;
// * To calibrate the altitude back to 0.0
bool altitudeCommand = true;
// * To set UTC time
bool missionTimeBool = false;
// * To release CanSat from container
bool probeUnlock = false;
// * To lock CanSat into container
bool probeLock = false;
// * To release nosecone (egg) from CanSat
bool eggUnlock = false;
// * To lock nosecone (egg) with CanSat
bool eggLock = false;
// * To actuate the right servo
bool RightServoBool = false;
// * To actuate the left servo
bool LeftServoBool = false;
// * To toggle reading from the SD Card
bool resetRead = false;
// * To toggle writing to the SD Card
bool resetWrite = true;
// * Variable assigned to incoming commands
String cmd;

// ===========|
// Simulation |
// ===========|
//* To assign variables upon entering simulation mode
bool sim = true;
// * To assign the basePressurePA to the first pressure value received
bool baseWait = true;
// * To change states per new pressure value received
bool simState = false;
// * simulated pressure value received
float simulationPressure = 101325.0;

//      |====================|
//      |SAMPLE DATA Section |
//      |====================|
// * Gyroscope variables (rad/s)
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;
// * Previous Gyroscope values
float Pgx = 0.0;
float Pgy = 0.0;
float Pgz = 0.0;
// * Gyroscope Timer
float Pgt = 0;
// * Acceleration variables (rad/s^2)
float ax;
float ay;
float az;
// * Altitude value from pressure formula
float altitude;
// * Temperature value
float temperature; 
// * Pressure value
float pressurePA;
// * Mode (flight or simulation)
bool determinedMode = true;
String mode;
// * To assign gps time to mission time, time variables
int Thour;
int Tminute;
int Tsecond;
// * variables used to measure remaining battery voltage
float battery_Voltage;
float rawSensorValue;
float ADC_REF = 5.0;
int ADC_RESOLUTION_CV = 4095;
int batteryPin = 14;
float VoltageInput;
// * variables used to measure the current
float current;
float Zero_Point;
float CurrentSensorValue;
float Vpin;
float Vout;
float CurrentPin = 15;
float Sensitivity = 0.185; // 185 mV / A

//      |=================|
//      |GPS DATA Section |
//      |=================|
// * New Latitude && Longitude
double New_Lat;
double New_Long;
// * GPS UTC Internal Clock
int GPS_Hour = 0;
int GPS_Minutes = 0;
int GPS_Seconds = 0;
// * GPS Altitude
float GPS_Altitude;
// * Satellites
int GPS_Satellites;

// =========|
// Distance |
// =========|
// * Target Cords
double latTarget;
double longTarget;
// * Current Cords
double latCurrent;
double longCurrent;
// * Delta distance
double deltaLat;
double deltaLong;
// * Variables for Distance formula equation
double a;
double c;
// * Radius of earth in meters
double R = 6371000;
// * Distance value
double Distance;

//      |=====================|
//      |FLIGHT STATE Section |
//      |=====================|
// * Set value once altitude drops (after ascending)
float apogeeHeight = 0;
// * Counter for each altitude below the previous
int ApogeeCounter = 0;
// * Value to hold previous altitude for comparison
float prevApogeeAlt = 0;
// * To actuate top servo to release CanSat from container after entering Descent
bool descentToProbe = false;
// * To actuate bottom servo to release nosecone from CanSat after entering Payload_Release
bool probeToPayload = false;
int ApogeeAmount = 10;
bool ApogeeBool = true;

//      |============================|
//      |ASYNCHRONOUS THREAD Section |
//      |============================|
// * To handle camera (on / off) toggling
bool cameraBool = true;

//      |================|
//      |SD CARD Section |
//      |================|
// * Timer variables for how often data is saved
unsigned long SDCARDTimer = 0.0;
unsigned long SDCARDInterval = 200.0;

//      |==================|
//      |TELEMETRY Section |
//      |==================|
// * Timer variables for how often telemetry is sent
unsigned long telemetryTime = 0;
const unsigned long telemetryInterval = 1000;
// * Number of packets sent
int packetCount = 0;
// * CanSat team Vortext assigned Team ID
int teamID = 1093;
// * To display the last sent command
String cmd_echo = "CMD_ECHO";

//      |=============================|
//      |ALTITUDE CALIBRATION Section |
//      |=============================|
// * Timer per pressure sample taken
unsigned long startTime;
// * Counter of pressure samples taken
int samplesTaken = 0;
// * summation of all pressure values sampled
float sum = 0;
// * number of samples taken
int validCount = 0;
// * temporary variable to check the pressure value is a valid number
float tempPressurePA;
// * The average pressure sampled, then used to calculate altitude
float basePressurePA = 0;

//      |============================|
//      |AUTONOMOUS CONTROLS Section |
//      |============================|
// * Target Cordinates
double T_Lat = 38.31015;
double T_Long = -78.73855;
double tLat;
double tLong;
// * Current Cords, assigned by New_Lat / New_Long
double C_Lat;
double C_Long;
double cLat;
double cLong;
// * Previous current cords
double P_Lat;
double P_Long;
double pLat;
double pLong;

// * All Target / Current / Previous degree cords turned into radians before trig is used
double cLatRad;
double cLongRad;
double tLatRad;
double tLongRad;
double pLatRad;
double pLongRad;
// * Delta's made for the atan2 function (to form bearings / vectors)
double deltaTLat;
double deltaTLong;
double deltaCLat;
double deltaCLong;
// * Directional bearings
double TargetBearing;
double CurrentBearing;
// * The difference in angle made from the two dierctional bearings
double error;
// * degree variables used for servo actuation
float RightTurn;
float LeftTurn;
// * Timer for servos, giving them time between each actuation
unsigned long ServoTimer = 0;
float ServoInterval = 1000;
// * Variable to not use servos after a processor restart has occured
bool Wait = false;

//      |======================|
//      |MISCELLANEOUS Section |
//      |======================|
// * Servo Mosfets
int TopSMosfet = 27;
int BottomSMosfet = 39;
int RightSMosfet = 41;
int LeftSMosfet = 21;
// * Camera Mosfets
int TopCamera = 31;
int BottomCamera = 30;


int init = 0;
};


// ================|
// Struct Variable |
// ================|
FlightData fd;

// ========================|
// State Machine Variables |
// ========================|
enum FlightState { 
  LAUNCH_PAD, 
  ASCENT, 
  APOGEE, 
  DESCENT, 
  PROBE_RELEASE,
  PAYLOAD_RELEASE, 
  LANDED 
  };

// =============|
// String Array |
// =============|
// * When printing the flightState, the enum position corresponds to the string flightState position (statenames[flightState])
const char* stateNames[7] = {
  "LAUNCH_PAD", 
  "ASCENT", 
  "APOGEE", 
  "DESCENT", 
  "PROBE_RELEASE",
  "PAYLOAD_RELEASE", 
  "LANDED"
  };
  // * Variable flightState is of the type FlightState (enum value)
 FlightState flightState = LAUNCH_PAD;

// ======================|
// Sensor Initialization |
// ======================|
// * Pressure && Temp
Adafruit_BMP3XX bmp;
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// * Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// * id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
// * GPS
SFE_UBLOX_GNSS myGNSS;

// ========================|
// SD File Initializations |
// ========================|
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
File State;
File resetReading;
File resetWriting;
File countFile;
File pressureSave;
File teleCom;
File simMode;
File simPressure;
File simReboot;
// * The 'i' is used in a for-loop to cycle through enum values until previous flightState is found
int i = 0;

// * Servos
PWMServo TopS;
PWMServo BottomS;
PWMServo LeftS;
PWMServo RightS;

// * Random
String Servoway;

//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |      ===================== Global Variables ======================        |
//                                                        |///////////////////////////////////////////////////////////////////////////|
// =====================|
// Function Declaration |
// =====================|
void commandChecker(FlightData &fd);
void sendTelemetry(FlightData &fd);
void sampleData(FlightData &fd);
void UpdateFlightState(FlightData &fd);
void autonomousControls(FlightData &fd);
void altitudeCalibration(FlightData &fd);
void newGPSData(FlightData &fd);
void saveToSDCARD(FlightData &fd);
void AsynchronousThread(void *arg);

//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |          ===================== VOID SETUP ======================          |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void setup() {

// ======================|
// Serial Initialization |
// ======================|
  Serial1.begin(115200);
  Serial.begin(115200);
  SD.begin(chipSelect);
  Wire.begin();
  delay(200);
  
// * Setting the ADC to 12-bit
  analogReadResolution(12);

// ===============================================|
// BMP 390 oversampling and filter initialization |
// ===============================================|
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);
  for(int k=0;k<20;k++){
    bmp.performReading();
    delay(50);
}
// ===================|
// GPS Initialization |
// ===================|
  myGNSS.begin();
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(2);
  myGNSS.setAutoPVT(true);
// =======================|
// SD Card Initialization |
// =======================|
  if(!SD.begin(BUILTIN_SDCARD)){
    Serial.println("SD initialization failed!");
  }
  dataFile = SD.open("Vortex_SD_CARD.txt", FILE_WRITE);
// ===================|
// BNO Initialization |
// ===================|
  bno.begin();
// ================================|
// Servo Teensy Pin Initialization |
// ================================|
  TopS.attach(12); // J11 // Mosfet Pin: 27
  BottomS.attach(9); // J10 // Mosfet Pin: 39
  LeftS.attach(3); // J8 // Mosfet Pin: 21
  RightS.attach(6); // J9 // Mosfet Pin: 41
  delay(100);
// =======================|
// MOSFETS Initialization |
// =======================|
  pinMode(fd.BottomSMosfet,OUTPUT);
  pinMode(fd.TopSMosfet,OUTPUT);
  pinMode(fd.RightSMosfet,OUTPUT);
  pinMode(fd.LeftSMosfet,OUTPUT);
  pinMode(fd.TopCamera,OUTPUT);
  pinMode(fd.BottomCamera,OUTPUT);
  digitalWrite(fd.TopSMosfet, HIGH);
  digitalWrite(fd.BottomSMosfet, HIGH);
  digitalWrite(fd.LeftSMosfet, HIGH);
  digitalWrite(fd.RightSMosfet, HIGH);
  delay(100);


// =================================|
// To Retrieve READ & WRITE toggles |
// =================================|
// * For toggling if we would like to retrieve SD Card data 
// * or not in the case of a processor restart.
// * Usually turned off for testing purposes.
  resetReading = SD.open("resetRead.txt", FILE_READ);
  if (resetReading){
    String Bool = "";
    while (resetReading.available()) {
      Bool += (char)resetReading.read();
    }
    Bool.trim();
    fd.resetRead = (Bool == "1");
    resetReading.close();
  }
  resetWriting = SD.open("resetWrite.txt", FILE_READ);
  if (resetWriting){
    String Write = "";
    while (resetWriting.available()) {
      Write += (char)resetWriting.read();
    }
    Write.trim();
    fd.resetWrite = (Write == "1");
    resetWriting.close();
  }
// ========================|
// To Retrieve flightState |
// ========================|
  State = SD.open("flightStateTest.txt", FILE_READ);
  if ((State) && (fd.resetRead)) {
    String nameOfState = "";
    while (State.available()) {
      nameOfState += (char)State.read();
    }
    nameOfState.trim();
    State.close();
    for (int fs = 0; fs < 7; fs++) {
      if (nameOfState == stateNames[fs]) {
        i = fs;
        flightState = FlightState(i);
        break;
      }
    }
    // * This is used to skip the next servo decision after a processor restart
    if ((flightState == DESCENT) || (flightState == PROBE_RELEASE)){
      fd.Wait = true;
    }

  } else {
    flightState = LAUNCH_PAD;
  }
// ========================|
// To Retrieve packetCount |
// ========================|
  countFile = SD.open("PacketNum.txt", FILE_READ);
  if ((countFile) && (fd.resetRead)) {
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
// ===========================|
// To Retrieve basePressurePA |
// ===========================|
  pressureSave = SD.open("Base.txt", FILE_READ);
  if ((pressureSave) && (fd.resetRead)) {
    String bp = "";
    while (pressureSave.available()) {
      bp += (char)pressureSave.read();
    }
    bp.trim();
    fd.basePressurePA = bp.toFloat();
    pressureSave.close();
  } else {
    altitudeCalibration(fd);
  }
// ==========================|
// To Retrieve CX_ON Command |
// ==========================|
  teleCom = SD.open("CX.txt", FILE_READ);
  if ((teleCom) && (fd.resetRead)) {
    String tele = "";
    while (teleCom.available()) {
      tele += (char)teleCom.read();
    }
    tele.trim();
    fd.telemetryStatus = (tele == "1");
    teleCom.close();
  }
// ============================|
// To Retrieve Simulation Mode |
// ============================|
  simMode = SD.open("Sim.txt", FILE_READ);
  if ((simMode) && (fd.resetRead)) {
    String simM = "";
    while (simMode.available()) {
      simM += (char)simMode.read();
    }
    simM.trim();
    fd.simulationMode = (simM == "1");
    simMode.close();
  }
// ============================================|
// To Retrieve SIM boolean for Simulation Mode |
// ============================================|
  simReboot = SD.open("simL.txt", FILE_READ);
  if ((simReboot) && (fd.resetRead)) {
    String si = "";
    while (simReboot.available()) {
      si += (char)simReboot.read();
    }
    si.trim();
    fd.sim = (si == "1");
    fd.baseWait = (si == "1");
    simReboot.close();
  }
// ===================================|
// To Retrieve the Simulated Pressure |
// ===================================|
  simPressure = SD.open("simP.txt", FILE_READ);
  if ((simPressure) && (fd.resetRead)) {
    String simp = "";
    while (simPressure.available()) {
      simp += (char)simPressure.read();
    }
    simp.trim();
    fd.simulationPressure = simp.toFloat();
    simPressure.close();
  } else {
    fd.simulationPressure = 101325.0;
  }




if (flightState == LAUNCH_PAD){
  TopS.write(15);
  Serial.println(" First Initialization ");
  delay(1000);
  TopS.write(0);
  delay(100);
  BottomS.write(90);
  delay(100);
  LeftS.write(0);
  delay(100);
  RightS.write(180);
  delay(100);
  Serial.println(" Second Initialization ");
  delay(1100);
  digitalWrite(fd.TopSMosfet, LOW);
  digitalWrite(fd.BottomSMosfet, LOW);
  digitalWrite(fd.LeftSMosfet, LOW);
  digitalWrite(fd.RightSMosfet, LOW);
}
// =================================|
// To Initialize Asynchronous Thread|
// =================================|
  threads.addThread(AsynchronousThread, &fd);

  altitudeCalibration(fd);

}

void loop() {
// * First check any incoming commands.
  commandChecker(fd);
// * Sample all sensor data.
  sampleData(fd);
// * In simulation mode, run FlightState per new simulation pressure. Otherwise run FlightState for flight mode continuously.
  if(fd.mode == "F"){
  UpdateFlightState(fd);
  } else {
    if(fd.simState){
      UpdateFlightState(fd);
      fd.simState = false;
      if(fd.ApogeeBool){
        fd.ApogeeAmount = 2;
        fd.ApogeeBool = false;
      }
    }
  }
// * Sample GPS and calculate Distance formula.
  newGPSData(fd);
// * With new gps data, calculate movement with servos.
if ((!(fd.C_Lat == fd.P_Lat)) && (flightState == PROBE_RELEASE)){
  autonomousControls(fd);
}
// * Send telemetry at 1 Hz.
  if(fd.telemetryStatus && ((millis() - fd.telemetryTime) >= fd.telemetryInterval)) {
    sendTelemetry(fd);
    fd.telemetryTime = millis();
    fd.packetCount ++;
  }
// * Save data to SD CARD at 10 Hz.
  if(((millis() - fd.SDCARDTimer) >= fd.SDCARDInterval)){
    saveToSDCARD(fd);
    fd.SDCARDTimer = millis();
  }
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |       ===================== Command Checker ======================        |
//                                                        |///////////////////////////////////////////////////////////////////////////|
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
      fd.simulationDisable = false;
      if(fd.simulationEnable){
        fd.sim = true;
      }

    } else if (fd.cmd == "CMD,1093,SIM,DISABLE") {
      fd.simulationDisable = true;
      fd.simulationActivate = false;
      fd.simulationEnable = false;
      fd.simulationMode = false;


    } else if (fd.cmd == "CMD,1093,CAL") {
      fd.altitudeCommand = true;
      fd.samplesTaken = 0;
      fd.sum = 0;
      fd.validCount = 0;
      fd.basePressurePA = 0;
      altitudeCalibration(fd);


    } else if (fd.cmd == "CMD,1093,SD,RESET"){
      if(SD.exists("flightStateTest.txt")){
        State.close();
        SD.remove("flightStateTest.txt");
      }
      if(SD.exists("PacketNum.txt")){
        countFile.close();
        SD.remove("PacketNum.txt");
      }
      if(SD.exists("Vortex_SD_CARD.txt")){
        dataFile.close();
        SD.remove("Vortex_SD_CARD.txt");
      }
      if(SD.exists("Base.txt")){
        pressureSave.close();
        SD.remove("Base.txt");
      }
      if(SD.exists("CX.txt")){
        teleCom.close();
        SD.remove("CX.txt");
      }
      if(SD.exists("Sim.txt")){
        simMode.close();
        SD.remove("Sim.txt");
      }
      if(SD.exists("simP.txt")){
        simPressure.close();
        SD.remove("simP.txt");
      }
      if(SD.exists("simL.txt")){
        simReboot.close();
        SD.remove("simL.txt");
      }

      flightState = LAUNCH_PAD;
      fd.altitudeCommand = true;
      fd.mode = "F";
      fd.simulationMode = false;
      fd.simulationEnable = false;
      fd.simulationActivate = false;
      fd.simulationDisable = true;
      fd.sim = true;
      fd.baseWait = true;
      fd.pressurePA = 0.0;
      fd.packetCount = 0;
      fd.Wait = false;
      fd.ApogeeCounter = 0;
      fd.prevApogeeAlt = 0;
      fd.apogeeHeight = 0;
      fd.telemetryStatus = false;
      digitalWrite(fd.TopSMosfet, HIGH);
      digitalWrite(fd.BottomSMosfet, HIGH);
      digitalWrite(fd.LeftSMosfet, HIGH);
      digitalWrite(fd.RightSMosfet, HIGH);
      delay(1000);
      TopS.write(0);
      delay(100);
      BottomS.write(90);
      delay(100);
      LeftS.write(0);
      delay(100);
      RightS.write(180);
      delay(1100);
      digitalWrite(fd.TopSMosfet, LOW);
      digitalWrite(fd.BottomSMosfet, LOW);
      digitalWrite(fd.LeftSMosfet, LOW);
      digitalWrite(fd.RightSMosfet, LOW);
      fd.basePressurePA = 0;
      fd.ApogeeAmount = 10;
      fd.ApogeeBool = true;
      fd.altitudeCommand = true;
      altitudeCalibration(fd);


      

    } else if (fd.cmd == "CMD,1093,SD,READ,ON"){
      fd.resetRead = true;

    } else if (fd.cmd == "CMD,1093,SD,READ,OFF") {
      fd.resetRead = false;
    
    } else if (fd.cmd == "CMD,1093,SD,WRITE,ON") {
      fd.resetWrite = true;

    } else if (fd.cmd == "CMD,1093,SD,WRITE,OFF") {
      fd.resetWrite = false;


  //--------Mechanism Actuation Commands---------
  
    } else if (fd.cmd == "CMD,1093,MEC,PROBE,UNLOCK") {
      fd.probeUnlock = true;

    } else if (fd.cmd == "CMD,1093,MEC,PROBE,LOCK") {
      fd.probeLock = true;

    } else if (fd.cmd == "CMD,1093,MEC,EGG,UNLOCK") {
      fd.eggUnlock = true;
    
    } else if (fd.cmd == "CMD,1093,MEC,EGG,LOCK") {
      fd.eggLock = true;

    } else if (fd.cmd == "CMD,1093,MEC,ACS,LEFT") {
      fd.LeftServoBool = true;

    } else if (fd.cmd == "CMD,1093,MEC,ACS,RIGHT") {
      fd.RightServoBool = true;

    }
    if(!fd.simulationMode){
      if((fd.simulationEnable) && (fd.simulationActivate) && (!fd.simulationDisable)){
        fd.simulationMode = true;
        if(fd.sim){
          fd.simulationPressure = 101325.0;
          fd.basePressurePA = 101325.0;
          flightState = LAUNCH_PAD;
          fd.sim = false;
          fd.ApogeeCounter = 0;
          fd.prevApogeeAlt = 0;
          fd.apogeeHeight = 0;

          
        }
      }
    }
    if(fd.cmd.startsWith("CMD,1093,SIMP,")){
      if (fd.simulationMode){
        fd.simulationPressure = fd.cmd.substring(fd.cmd.lastIndexOf(",") + 1).toFloat();
        fd.simState = true;
        if(fd.baseWait){
          fd.basePressurePA = fd.simulationPressure;
          fd.baseWait = false;
        }
      }
    }
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |     ===================== Altitude Calibration ======================     |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void altitudeCalibration(FlightData &fd) {
  if(fd.altitudeCommand) {
//      --------- Base Pressure Calibration ---------
/*
  fd.startTime = millis();

  while(fd.samplesTaken < 6) {
    if ((millis() - fd.startTime) >= 80) {
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
  */
  if(fd.basePressurePA == 0){
  fd.startTime = millis();
  while (!((millis() - fd.startTime) > 2000)){
    if (bmp.performReading()) {
    fd.basePressurePA = bmp.pressure;
    Serial.println(fd.basePressurePA);
  } else{
    delay(1500);
  }
  fd.altitudeCommand = false;
    }
  }
}
}

//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |      ===================== Sampling Section ======================        |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void sampleData(FlightData &fd) {

if(fd.simulationMode){
    fd.pressurePA = fd.simulationPressure;
    if(bmp.performReading()) {
        fd.temperature = bmp.temperature;
    }
}
else{
    if(bmp.performReading()) {
        fd.pressurePA = bmp.pressure;
        fd.temperature = bmp.temperature;
    }
}

fd.altitude = 44330.0 * (1 - pow(fd.pressurePA / fd.basePressurePA, 0.1903));
  if (fd.altitude > 10000){
    fd.altitude = 0.0;
  }

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
fd.battery_Voltage = (fd.VoltageInput * 3) - 3.45;

// Current Sensor
fd.Zero_Point = 0.5; //2.5 volts accoring to Pololu datasheet
fd.CurrentSensorValue = analogRead(fd.CurrentPin);
fd.Vpin = (fd.CurrentSensorValue / fd.ADC_RESOLUTION_CV) * 3.3;
fd.Vout = fd.Vpin * 2.0;
fd.current = (fd.Vout - fd.Zero_Point) / fd.Sensitivity;
fd.current = fd.current - 0.8;

//      ----Mode Determiner Code----
if(fd.simulationMode) {
  fd.determinedMode = false;
} else {
  fd.determinedMode = true;
}
fd.mode = fd.determinedMode ? "F" : "S";

//      ----UTC Time----
if (fd.missionTimeBool){

  fd.Thour =  myGNSS.getHour() - 5;
  fd.Tminute = myGNSS.getMinute();
  fd.Tsecond = myGNSS.getSecond();
  }
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |        ===================== Flight State ======================          |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void UpdateFlightState(FlightData &fd) {

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
        } else if(fd.ApogeeCounter >= fd.ApogeeAmount){
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
        fd.descentToProbe = true;
      }
      break;
    case PROBE_RELEASE:
      if (fd.altitude <= 2.5){
        flightState = PAYLOAD_RELEASE;
        fd.probeToPayload = true;
      }
      break;
    case PAYLOAD_RELEASE:

      if ((fd.altitude > -10) && (fd.altitude < 2)){
        flightState = LANDED;

      }
      break;
    case LANDED:
      break;
  }
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |          ===================== GPS Data ======================            |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void newGPSData(FlightData &fd){

if(myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)){
  fd.P_Lat = fd.C_Lat;
  fd.P_Long = fd.C_Long;
  fd.New_Lat = (myGNSS.getLatitude() / (pow(10,7)));
  fd.New_Long = (myGNSS.getLongitude() / (pow(10,7)));
  fd.GPS_Altitude = myGNSS.getAltitude();
  fd.GPS_Hour = myGNSS.getHour();
  fd.GPS_Minutes = myGNSS.getMinute();
  fd.GPS_Seconds = myGNSS.getSecond();
  fd.GPS_Satellites = myGNSS.getSIV();

  fd.C_Lat = fd.New_Lat;
  fd.C_Long = fd.New_Long;


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
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |     ===================== Autonomous Controls ======================      |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void autonomousControls(FlightData &fd) {

// Current GPS position
fd.cLat = fd.C_Lat;
fd.cLong = fd.C_Long;
// Target GPS position
fd.tLat = fd.T_Lat;
fd.tLong = fd.T_Long;
// Previous GPS position
fd.pLat = fd.P_Lat;
fd.pLong = fd.P_Long;
// Convert all decimal degrees into radians
fd.cLatRad = fd.cLat * (PI / 180);
fd.cLongRad = fd.cLong * (PI / 180);
fd.tLatRad = fd. tLat * (PI / 180);
fd.tLongRad = fd.tLong * (PI / 180);
fd.pLatRad = fd.pLat * (PI / 180);
fd.pLongRad = fd.pLong * (PI / 180);
// Vector / delta distances
fd.deltaTLat = fd.tLatRad - fd.cLatRad;
fd.deltaTLong = (fd.tLongRad - fd.cLongRad) * (cos((fd.tLatRad + fd.cLatRad) / 2));
fd.deltaCLat = fd.cLatRad - fd.pLatRad;
fd.deltaCLong = (fd.cLongRad - fd.pLongRad) * (cos((fd.cLatRad + fd.pLatRad) / 2));
/* 
Computes the bearings (in radians) for navigation using the atan2 function
  TargetBearing: The angle toward target location
  CurrentBearing: angle of current trajectory
*/
fd.TargetBearing = atan2(fd.deltaTLong, fd.deltaTLat);
fd.CurrentBearing = atan2(fd.deltaCLong, fd.deltaCLat);
// Convert back to degrees
fd.TargetBearing = fd.TargetBearing * (180 / PI);
fd.CurrentBearing = fd.CurrentBearing * (180 / PI);
// The interval is between [-180, 180], convert to [0, 360]
if (fd.TargetBearing < 0) {
  fd.TargetBearing = fd.TargetBearing + 360;
}
if (fd.CurrentBearing < 0){
  fd.CurrentBearing = fd.CurrentBearing + 360;
}
// The error is calculated to be the difference in angle measurement
fd.error = fd.TargetBearing - fd.CurrentBearing;
// For steering purposes, we need to take the fastest turn. 
// 0 and 360 are the same direction. 
// Instead of going left 340 degrees, we would rather go right 20 degrees
// This converts our [0, 360] Interval back to [-180, 180]
if (fd.error <= -180){
  fd.error = fd.error + 360;
} else if (fd.error > 180){
  fd.error = fd.error - 360;
}
// if error is > 0 then Turn Right: This is found through testing 
// if error is < 0 then Turn Left: This is found through testing
// For steering, we would want to actuate based on the error.
// Also allowing one servo to be turned at a time. This will ensure a safer navigation.
if (fd.Distance <= 6){
  LeftS.write(180);
} else {
if (fd.error >= 0){
  fd.RightTurn = ((180 - fd.error)*(1/36)) + 175;
  fd.LeftTurn = fd.error - 180;
} else if (fd.error < 0){
  fd.RightTurn = 180 - fd.error;
  fd.LeftTurn = abs(fd.error) * (1/36);
}


if ((fd.RightTurn > 180) || (fd.RightTurn < 0)){
  Servoway = "Left";
} else {
  Servoway = "Right";
}

// To turn the Servos: 
    LeftS.write(fd.LeftTurn);
    RightS.write(fd.RightTurn);
  }
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |          ===================== Telemetry ======================           |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void sendTelemetry(FlightData &fd) {
  // All of the Serial1 prints go straight to the UART connection: XBEE
  // All of the Serial prints go straight to the Terminal for testing purposes

  // * XBEE:
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

  // * Terminal:
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
  Serial.println(fd.cmd_echo);
}
//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |           ===================== SD CARD ======================            |
//                                                        |///////////////////////////////////////////////////////////////////////////|

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
  dataFile.print(fd.apogeeHeight, 2); dataFile.print(",");
  dataFile.print(fd.basePressurePA, 2); dataFile.print(",");
  dataFile.print(fd.Distance, 3); dataFile.print(",");
  dataFile.print(fd.P_Lat, 5); dataFile.print(",");
  dataFile.print(fd.P_Long, 5); dataFile.print(",");
  dataFile.println(fd.cmd_echo);
  dataFile.flush();
  dataFile.close();


// =========================|
// PROCESSOR RESTART SAVING |
// =========================|


// * Save the resetRead boolean for toggling whether we retrieve data when powered back on.
  resetReading = SD.open("resetRead.txt", FILE_WRITE);
  if (resetReading) {
    resetReading.seek(0);
    resetReading.print(fd.resetRead);
    resetReading.close();
  }
// * Save the resetWrite boolean for toggling whether we save data to pull back from
  resetWriting = SD.open("resetWrite.txt", FILE_WRITE);
  if (resetWriting) {
    resetWriting.seek(0);
    resetWriting.print(fd.resetWrite);
    resetWriting.close();
  }
if (fd.resetWrite){
  // * Continually save the current flightState
    State = SD.open("flightStateTest.txt", FILE_WRITE);
    if (State) {
      State.seek(0);
      State.print(stateNames[flightState]);
      State.truncate(String(stateNames[flightState]).length());
      State.close();
    }
  // * Continually save the last packet number sent.
    countFile = SD.open("PacketNum.txt", FILE_WRITE);
    if (countFile) {
      countFile.seek(0);
      countFile.print(fd.packetCount);
      countFile.truncate(String(fd.packetCount).length());
      countFile.close();
    }
  // * Save the base pressure.
    pressureSave = SD.open("Base.txt", FILE_WRITE);
    if (pressureSave) {
      pressureSave.seek(0);
      pressureSave.print(fd.basePressurePA);
      pressureSave.truncate(String(fd.basePressurePA).length());
      pressureSave.close();
    }
    simPressure = SD.open("simP.txt", FILE_WRITE);
    if (simPressure) {
      simPressure.seek(0);
      simPressure.print(fd.simulationPressure);
      simPressure.truncate(String(fd.simulationPressure).length());
      simPressure.close();
    }
  // * Save the telemetryStatus (on / off)
    teleCom = SD.open("CX.txt", FILE_WRITE);
    if (teleCom) {
      teleCom.seek(0);
      teleCom.print(fd.telemetryStatus);
      teleCom.close();
    }
  // * Save the simulationMode status (F / S)
    simMode = SD.open("Sim.txt", FILE_WRITE);
    if (simMode) {
      simMode.seek(0);
      simMode.print(fd.simulationMode);
      simMode.close();
    }
    simReboot = SD.open("simL.txt", FILE_WRITE);
    if (simReboot) {
      simReboot.seek(0);
      simReboot.print(fd.sim);
      simReboot.close();
    }
  }
}

//                                                        |///////////////////////////////////////////////////////////////////////////|
//                                                        |     ===================== Asynchronous THREAD ======================      |
//                                                        |///////////////////////////////////////////////////////////////////////////|
void AsynchronousThread(void *arg) {
  FlightData &fd = *(FlightData*)arg;

while(1){
  if(fd.descentToProbe){
        digitalWrite(fd.TopSMosfet, HIGH);
        threads.delay(200);
        TopS.write(15);
        threads.delay(900);
        digitalWrite(fd.TopSMosfet, LOW);
        digitalWrite(fd.RightSMosfet, HIGH);
        digitalWrite(fd.LeftSMosfet, HIGH);
        threads.delay(100);
        fd.descentToProbe = false;
  }
  if(fd.probeToPayload){
        digitalWrite(fd.BottomSMosfet, HIGH);
        threads.delay(200);
        BottomS.write(10);
        threads.delay(1000);
        digitalWrite(fd.BottomSMosfet, LOW);
        digitalWrite(fd.RightSMosfet, LOW);
        digitalWrite(fd.LeftSMosfet, LOW);
        threads.delay(100);
        fd.probeToPayload = false;
  }
  if(fd.probeUnlock){
      digitalWrite(fd.TopSMosfet, HIGH);
      threads.delay(200);
      TopS.write(15);
      threads.delay(1000);
      digitalWrite(fd.TopSMosfet, LOW);
      fd.probeUnlock = false;
  }
  if(fd.probeLock){
      digitalWrite(fd.TopSMosfet, HIGH);
      threads.delay(200);
      TopS.write(0);
      threads.delay(2000);
      digitalWrite(fd.TopSMosfet, LOW);
      fd.probeLock = false;    
  }
  if(fd.eggUnlock){
      digitalWrite(fd.BottomSMosfet, HIGH);
      threads.delay(300);
      BottomS.write(10);
      threads.delay(1000);
      digitalWrite(fd.BottomSMosfet, LOW);
      fd.eggUnlock = false;
  }
  if(fd.eggLock){
      digitalWrite(fd.BottomSMosfet, HIGH);
      threads.delay(300);
      BottomS.write(90);
      threads.delay(800);
      digitalWrite(fd.BottomSMosfet, LOW);
      fd.eggLock = false;
  }
  if(flightState == LANDED){
    if(fd.cameraBool){
      threads.delay(1000);
      digitalWrite(fd.TopCamera, LOW);
      digitalWrite(fd.BottomCamera, LOW);
      fd.cameraBool = false;
    }
  }
  if(fd.LeftServoBool){
    digitalWrite(fd.LeftSMosfet, HIGH);
    threads.delay(200);
    LeftS.write(180);
    threads.delay(1000);
    LeftS.write(0);
    threads.delay(500);
    digitalWrite(fd.LeftSMosfet, LOW);
    fd.LeftServoBool = false;
  }
  if(fd.RightServoBool){
      digitalWrite(fd.RightSMosfet, HIGH);
      threads.delay(200);
      RightS.write(0);
      threads.delay(1000);
      RightS.write(180);
      threads.delay(500);
      digitalWrite(fd.RightSMosfet, LOW);
      fd.RightServoBool = false;
  }
  threads.yield();
  }
}



