//#include "../ublox-lib/src/SparkFun_Ublox_Arduino_Library.h"

//#include "../../Documents/Arduino/libraries/ublox-master/src/SparkFun_Ublox_Arduino_Library.h"
//#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier

//---------LORA
//#include <SPI.h>
#include "./LoRa.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#define SS 18
#define RST 14
#define DI0 26
#define BAND 915E6
//-------------

//---------DISPLAY
#include "SSD1306.h"
//#include "../../Documents/Arduino/libraries/ESP8266_and_ESP32_Oled_Driver_for_SSD1306_display/src/SSD1306.h"
//#include "./OLEDDisplayUi.h"
SSD1306  display(0x3c, 4, 15);
 #include "./images.h"
//----------------


//-------------IMU  THIS VERSION DOES NOT USE AN IMU
#include "EEPROM.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
//Adafruit_BNO055 IMU;
//-----------------
void ScanForI2CDevices();

//---------GPS
SFE_UBLOX_GPS GPS1;   SFE_UBLOX_GPS* ROVERGPS;
SFE_UBLOX_GPS GPS2;   SFE_UBLOX_GPS* BASEGPS;

float baselineDistanceCentimeters = 19;





//------------

uint8_t verticalPosition = 0;

void setup() {
  //----------OLED SETUP
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  delay(50);
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,0, "Initializing....");
  verticalPosition+=10;
  display.drawProgressBar(0, 50, 120, 10, 10);
  display.display();
  //-------------------

  
  //----------SERIAL
  Serial.begin(115200);
  Serial.println("Serial port enabled...");
  //----------------

  //----------GPS
  Wire1.begin(21,22);
  delay(500);
  if(GPS1.begin(Wire1, 0x40) == false)
  {
    display.drawString(0, verticalPosition, "GPS1 Connection Failed");
    Serial.println("Failed to connect to GPS1");
    
  }
  else
  {
    display.drawString(0, verticalPosition, "GPS1 Connected!");
    Serial.println("Connected to GPS1!");
  }
  delay(500);
  /*
  GPS1.setI2COutput(COM_TYPE_NMEA); //Set the I2C port to output UBX only (turn off NMEA noise)
  GPS1.setNavigationFrequency(2); //Produce two solutions per second
  GPS1.setAutoPVT(true); //Tell the GPS to "send" each solution
  GPS1.saveConfiguration(); //Save the current settings to flash and BBR
  */
  verticalPosition += 10;
  
  display.drawProgressBar(0, 50, 120, 10, 25);
  delay(250);
  
  if(GPS2.begin(Wire1, 0x42) == false)
  {
    display.drawString(0, verticalPosition, "GPS2 Connection Failed");
    Serial.println("Failed to connect to GPS2");
    
  }
  else
  {
    display.drawString(0, verticalPosition, "GPS2 Connected!");
    Serial.println("Connected to GPS2!");
  }
  delay(500);
  /*
  GPS2.setI2COutput(COM_TYPE_NMEA); //Set the I2C port to output UBX only (turn off NMEA noise)
  GPS2.setNavigationFrequency(2); //Produce two solutions per second
  GPS2.setAutoPVT(true); //Tell the GPS to "send" each solution
  GPS2.saveConfiguration(); //Save the current settings to flash and BBR
*/
  //GPS2.setNMEAOutputPort(Serial);
  //GPS1.setNMEAOutputPort(Serial);
  GPS1.setI2COutput(COM_TYPE_UBX);
  GPS2.setI2COutput(COM_TYPE_UBX);
  //GPS1.enableDebugging();
  //GPS2.enableDebugging();
  GPS1.saveConfiguration();
  GPS2.saveConfiguration();
  verticalPosition += 10;
  display.drawProgressBar(0, 50, 120, 10, 45);
  display.display();
  delay(250);
  

  Serial.println(GPS2.getAltitude());
  Serial.println(GPS2.getHour());
  Serial.println(GPS2.getProtocolVersion());
   Serial.println(GPS1.getAltitude());
  Serial.println(GPS1.getHour());
  Serial.println(GPS1.getProtocolVersion());

  GPS1.setNavigationFrequency(2); //Produce two solutions per second
  GPS1.setAutoPVT(true); //Tell the GPS to "send" each solution

  GPS2.setNavigationFrequency(2); //Produce two solutions per second
  GPS2.setAutoPVT(true); //Tell the GPS to "send" each solution
  

  
  ROVERGPS = &GPS1;
  BASEGPS = &GPS2;

  /*if(sendTMODEDisable())
    Serial.println("TMODE disabled");
  else
    Serial.println("Failed to disable TMODE");*/


  GPS1.saveConfiguration(); //Save the current settings to flash and BBR
  GPS2.saveConfiguration(); //Save the current settings to flash and BBR
  //--------------

  //----------IMU
  /*
  EEPROM.begin(64);
  IMU = Adafruit_BNO055(5,0x28, &Wire1);
  if(!IMU.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    
  }
  else
  {
    Serial.println("Successfully connected to IMU");
    displaySensorDetails();
    displaySensorStatus();
  }
  IMU.setExtCrystalUse(true);
  int eeAddress = 1;
  long imuID;
  bool foundCalib = false;
  EEPROM.get(eeAddress, imuID);

  Serial.print("IMU ID: "); Serial.println(imuID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  IMU.getSensor(&sensor);
  if(imuID != sensor.sensor_id)
  {
    Serial.println("No configuration data found for IMU");
  }
  else
  {
    Serial.println("Found configuration data for IMU");
    eeAddress+= sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    displaySensorOffsets(calibrationData);
    Serial.println("Restoring calibration data to the IMU...");
    IMU.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into IMU!");
    foundCalib = true;
  }
      sensors_event_t event;
    IMU.getEvent(&event);
        // always recal the mag as It goes out of calibration very often 
    if (foundCalib)
    {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!IMU.isFullyCalibrated())
        {
            IMU.getEvent(&event);
            delay(100);
        }
    }
    else
    {
      Serial.println("Please Calibrate IMU: ");
      while (!IMU.isFullyCalibrated())
      {
          IMU.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          // Optional: Display calibration status 
          displayCalStatus();

          // New line for the next sample 
          Serial.println("");

          // Wait the specified delay before requesting new data 
          delay(100);
      }
    }
        Serial.println("IMU fully calibrated!");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    IMU.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);
    
    Serial.println("--------------------------------");
    Serial.println("\n\nStoring IMU calibration data to EEPROM...");

    eeAddress = 1;
    IMU.getSensor(&sensor);
    imuID = sensor.sensor_id;

    EEPROM.put(eeAddress, imuID);
    Serial.print("IMU ID: "); Serial.println(imuID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    EEPROM.commit();
    Serial.println("IMU calibration stored to EEPROM.");*/
  //------------
  
  
  
  //----------LORA
  
  SPI.begin(5,19,27,18);
  delay(100);
  LoRa.setPins(SS,RST,DI0);
  
  display.drawProgressBar(0,50, 120, 10, 70);
  display.display();
  delay(100);
  
  if (!LoRa.begin(BAND, true)) 
  {
    Serial.println("Starting LoRa failed!");
    display.drawString(0, verticalPosition, "LoRa Connection Failed");
  }
  else
  {
    display.drawString(0, verticalPosition, "LoRa Connected!");
  }
  verticalPosition+=10;
  display.drawProgressBar(0, 50, 120, 10, 90);
  display.display();
  delay(100);
  //--------------

  //display.drawString(0, verticalPosition, "Starting.....");
  display.drawProgressBar(0, 50, 120, 10, 100);
  display.display();
  delay(1000);
  
  display.clear();
  display.drawXbm(7, 0, avl_logo_width, avl_logo_height, avl_logo_bytes);
  display.display();
  delay(1000);

  

  
}


void loop() {

  verticalPosition = 0;

  int GPS1FixType = 0;
  int GPS2FixType = 0;
  int GPS1SIV = 0;
  int GPS2SIV = 0;

  display.clear();
  if(GPS1.isConnected())
  {
    GPS1FixType = GPS1.getFixType();
    GPS1SIV = GPS1.getSIV();
    Serial.print("GPS1 Fix: "); Serial.println(GPS1.getFixType());
    Serial.print("GPS1 SIV: "); Serial.println(GPS1.getSIV());
  }
  if(GPS2.isConnected())
  {
    GPS2FixType = GPS2.getFixType();
    GPS2SIV = GPS2.getSIV();
    Serial.print("GPS2 Fix: "); Serial.println(GPS2.getFixType());
    Serial.print("GPS2 SIV: "); Serial.println(GPS2.getSIV());
  }

  if(GPS1.isConnected() == false || GPS2.isConnected() == false )
  {
    ScanForI2CDevices();
    delay(2000);
    display.drawString(0, 30, "Could not connect to GPS modules... ");
  }



  double latitude1=0;
  double longitude1=0;
  int32_t gpsLat1=0;
  int32_t gpsLon1=0;
  int32_t gpsLat2=0;
  int32_t gpsLon2=0;
  double latitude2=0;
  double longitude2=0;
  long altitude2=0;
  long altitude1=0;

  bool GPS1Valid=false;
  bool GPS2Valid=false;
  
  
  bool roverHasAttitude = false;
  float nNED;
  float eNED;
  float dNED;

  int8_t relPosHPN;
  int8_t relPosHPE;
  int8_t relPosHPD;

  float nVect;
  float eVect;
  float dVect;

  float heading;
  float vectLen;
  float elevationAngle;

  if(ROVERGPS->getPVT() || true)
  {
    uint32_t posnAccuracy =  ROVERGPS->getPositionAccuracy();
    uint8_t solnType = ROVERGPS->getCarrierSolutionType(); //0=No solution, 1=Float solution, 2=Fixed solution
    Serial.print("Rover Solution Type: "); Serial.println(solnType);
    Serial.print("Position Accuracy: "); Serial.println(posnAccuracy);
    if(solnType > 0)
    {
      
    }
    if(ROVERGPS->getRELPOSNED(2000))
    {
      roverHasAttitude = true;
      nNED = ROVERGPS->relPosInfo.relPosN;
      eNED = ROVERGPS->relPosInfo.relPosE;
      dNED = ROVERGPS->relPosInfo.relPosD;

      relPosHPN = ROVERGPS->relPosInfo.relPosHPN;
      relPosHPE = ROVERGPS->relPosInfo.relPosHPE;
      relPosHPD = ROVERGPS->relPosInfo.relPosHPD;

      nVect = ROVERGPS->relPosInfo.relPosN + 0.01*ROVERGPS->relPosInfo.relPosHPN;
      eVect = ROVERGPS->relPosInfo.relPosE + 0.01*ROVERGPS->relPosInfo.relPosHPE;
      dVect = ROVERGPS->relPosInfo.relPosD + 0.01*ROVERGPS->relPosInfo.relPosHPD;

      bool isRelPosValid = ROVERGPS->relPosInfo.relPosValid;
      bool isMoving = ROVERGPS->relPosInfo.isMoving;      

      heading = 180 * atan2(eVect, nVect)/3.14159265;
      vectLen = sqrt(nVect*nVect + eVect*eVect + dVect*dVect);
      elevationAngle = atan2(dVect,(sqrt(nVect*nVect + eVect*eVect)))*180/3.14159265358;

      Serial.println("Valid relative position!!!");
      Serial.print(nNED); Serial.print("     "); Serial.print(eNED); Serial.print("     "); Serial.println(dNED);
      Serial.print(nVect); Serial.print("     "); Serial.print(eVect);  Serial.print("     "); Serial.println(dVect);
      Serial.print(ROVERGPS->relPosInfo.carrSoln); Serial.print("    "); Serial.print(ROVERGPS->relPosInfo.relPosHPLength);
      Serial.print("     ");  Serial.println(ROVERGPS->relPosInfo.relPosLength); Serial.print("     "); Serial.println(ROVERGPS->relPosInfo.relPosHeading); 
      Serial.print("Heading: "); Serial.print(heading); Serial.print("      Length: "); Serial.print(vectLen); Serial.print("     Elevation: ");
      Serial.println(elevationAngle);

      
    }
  }
  if(BASEGPS->getPVT())
  {
    Serial.print("BASE PVT: "); Serial.println(BASEGPS->getHeading());
    /*Serial.println("BASE PVT is ready");
     if(BASEGPS->getRELPOSNED(2000))
    {
      float nNED = BASEGPS->relPosInfo.relPosN;
      float eNED = BASEGPS->relPosInfo.relPosE;
      float dNED = BASEGPS->relPosInfo.relPosD;

      int8_t relPosHPN = BASEGPS->relPosInfo.relPosHPN;
      int8_t relPosHPE = BASEGPS->relPosInfo.relPosHPE;
      int8_t relPosHPD = BASEGPS->relPosInfo.relPosHPD;

      float nVect = BASEGPS->relPosInfo.relPosN + 0.01*BASEGPS->relPosInfo.relPosN;
      float eVect = BASEGPS->relPosInfo.relPosE + 0.01*BASEGPS->relPosInfo.relPosE;
      float dVect = BASEGPS->relPosInfo.relPosD + 0.01*BASEGPS->relPosInfo.relPosD;

      bool isRelPosValid = BASEGPS->relPosInfo.relPosValid;
      bool isMoving = BASEGPS->relPosInfo.isMoving;      

      Serial.println("Valid relative position!!!");
      Serial.print(nNED); Serial.print("     "); Serial.print(eNED); Serial.print("     "); Serial.println(dNED);
      Serial.print(nVect); Serial.print("     "); Serial.print(eVect);  Serial.print("     "); Serial.println(dVect);
      Serial.print(BASEGPS->relPosInfo.carrSoln); Serial.print("    "); Serial.print(BASEGPS->relPosInfo.relPosHPLength);
      Serial.print("     ");  Serial.println(BASEGPS->relPosInfo.relPosLength); Serial.print("     "); Serial.println(BASEGPS->relPosInfo.relPosHeading); 

*/
      
  }
  


  //------------------------

  //------SEND LORA PACKETS
  

  if(GPS1Valid && false)
  {
    String packet1 = "GPS1: Fix=";
    packet1 = packet1 + GPS1FixType + ",SIV=" + GPS1SIV + ",LAT=" + gpsLat1 + ",LON=" + gpsLon1 + ",ALT=" + altitude1;
    LoRa.println(packet1);
  }
  if(GPS2Valid && false)
  {
    String packet2 = "GPS2: Fix=";
    packet2 = packet2 + GPS2FixType + ",SIV=" + GPS2SIV + ",LAT=" + gpsLat2 + ",LON=" + gpsLon2 + ",ALT=" + altitude2;
    LoRa.println(packet2);
  }
  if(ROVERGPS->isConnected() && false)
  {
    String packet2 = "GPS2: Fix=";
    int32_t lat = ROVERGPS->getLatitude();
    int32_t lon = ROVERGPS->getLongitude();
    uint8_t fix = ROVERGPS->getFixType();
    int32_t alt = ROVERGPS->getAltitudeMSL();
    packet2 = packet2 + fix + ",LAT=" + lat + ",LON=" + lon + ",ALT=" + alt;
    Serial.print("LORA: "); Serial.println(packet2);
    LoRa.println(packet2);
  }
  if(GPS1Valid && GPS2Valid && false)
  {
    String packet3 = "DIFFERENCE: LAT=";
    long diffLat = gpsLat1-gpsLat2;
    long diffLon = gpsLon1-gpsLon2;
    long diffAlt = altitude1 - altitude2;
    packet3 = packet3 + diffLat + ",LON=" + diffLon + ",ALT=" + diffAlt;
    Serial.println(packet3);
    LoRa.println(packet3);
  }

  if(roverHasAttitude && !(heading == 0 || vectLen == 0))
  {
    LoRa.beginPacket();
    uint8_t carrierSoln = ROVERGPS->carrierSolution;
    String packet4 = "RELATIVE: N=";
    packet4 = packet4 + nNED + ",E=" + eNED + ",D=" + dNED + ",NVECT=" + nVect + ",EVECT=" + eVect + ",DVECT=" + dVect;
    display.clear();
    String displayString = "";
    displayString = displayString + "N=" + nNED + " E=" + eNED + " D=" + dNED;
    display.drawString(0,0, displayString);
    String displayString2 = "";
    displayString2 = displayString2 + "N=" + nVect + " E=" + eVect + " D=" + dVect;
    display.drawString(0, 15, displayString2);
    String displayString3 = "H=";
    displayString3 = displayString3 + heading + " L=" + vectLen + " EL=" + elevationAngle;
    display.drawString(0, 30, displayString3);
    String displayString4 = "SOLN=";
    displayString4 = displayString4 + carrierSoln;
    display.drawString(0, 45, displayString4);
    display.display();
   
    packet4 = packet4 + ",H=" + heading + ",LEN=" + vectLen + ",EL=" + elevationAngle + ",SOL=" + carrierSoln;
    Serial.print("LORA: "); Serial.println(packet4);
    LoRa.println(packet4);
    LoRa.endPacket();

  }
  

  
  //-----------------IMU
  /*LoRa.beginPacket();
  imu::Vector<3> magVector = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  delay(100);
  imu::Vector<3> gravVector = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  if(!(magVector.x() == 0 && magVector.y() == 0))
  {
    String packet5 = "MAG: ";
    packet5 = packet5 + "X=" + magVector.x() + ",Y=" + magVector.y() + ",Z=" + magVector.z();
    
    LoRa.println(packet5);
  }
  if(!(gravVector.x() == 0 && gravVector.y() == 0))
  {
    String packet6 = "ORIENTATION: ";
    packet6 = packet6 + "X=" + gravVector.x() + ",Y=" + gravVector.y() + ",Z=" + gravVector.z();
    LoRa.println(packet6);
  }

  //calculate magnetic heading
  if((gravVector.x() != 0 || gravVector.y() != 0) && (magVector.x() != 0 || magVector.y() != 0))
  {
    double magneticHeading = -999;
    if(abs(gravVector.x()) > abs(gravVector.y()) && abs(gravVector.x()) > abs(gravVector.y()))
    {
      //orientation = standing up
      magneticHeading = atan2(magVector.y(), magVector.z())*180/3.14159265;
      
    }
    else if(abs(gravVector.z()) > abs(gravVector.y()) && abs(gravVector.z()) > abs(gravVector.x()))
    {
      //orientation = lying down
      magneticHeading = atan2(magVector.x(), magVector.y())*180/3.14159265;
    }

    if(magneticHeading != -999)
    {
      String packet7 = "MAGHEADING: ";
      packet7 = packet7 + magneticHeading + " degrees";
      LoRa.println(packet7);
    }
  }
  LoRa.endPacket();

  //----------------------

  
  display.display();
  delay(3000);
}

bool sendTMODEDisable()
{
  if(BASEGPS->setVal(0x20030001, 0))
    return true;
  else
    return false;
}*/

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
/*void displaySensorStatus(void)
{
  //Get the system status values (mostly for debugging purposes) 
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  IMU.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor 
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}*/

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
/*void displayCalStatus(void)
{
  // Get the four calibration values (0..3) 
  // Any sensor data reporting 0 should be ignored, 
  // 3 means 'fully calibrated" 
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  IMU.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0 
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  // Display the individual values 
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  IMU.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);*/
}

void ScanForI2CDevices()
{
  Serial.println("Scanning for I2C devices");
    int address;
    int nDevices = 0;
    for (address = 1; address < 127; address++ )
    {
      Wire1.beginTransmission(address);
      byte error = Wire1.endTransmission();
  
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        Serial.println("  !");
  
        nDevices++;
      }
      else if (error == 4)
      {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
          Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
}
