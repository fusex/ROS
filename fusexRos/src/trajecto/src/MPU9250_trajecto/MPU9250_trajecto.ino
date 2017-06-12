#include "MPU9250.h"
#include "quaternionFilters.h"


/* MPU9250 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>




#define SerialDebug false  // Set to true to get formated Serial output for debugging or false to get raw data

// Pin definitions

// I2C
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

//Tiny GPS
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

MPU9250 myIMU;
// The TinyGPS++ object
TinyGPSPlus gps;
float lati;
float longi;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

double gpsTimer =0;

void setup()
{
    //////// GPS///////////
  ss.begin(GPSBaud);
  
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

  myIMU.initMPU9250();

  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.factoryMagCalibration);

  //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

  //////// GPS///////////
  ss.begin(GPSBaud);
  longi = 0;
  lati  = 0;
  


} // void setup()

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();



    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    // Following data from manual calibration were extracted
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();


//  myIMU.magBias[0]  = 173.275;
  //myIMU.magBias[1]  = 155.415;
 // myIMU.magBias[2] = -886,805;
 

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] ;
    //- myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] ;
    //-myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] ;
    //-myIMU.magBias[2];


    //************** manual calibration ****************
    //myIMU.mx *= myIMU.magScale[0];
    //myIMU.my *= myIMU.magScale[1];
    //myIMU.mz *= myIMU.magScale[2];


  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;




////////// GPS
smartDelay(10);
if (gps.location.isValid())
{
  lati = gps.location.lat();
  longi = gps.location.lng();
}
  
  // update LCD once per half-second independent of read rate
  if (myIMU.delt_t > 10)
  {
    // ***** Data transmitted according to the following pattern ****
    //      [ax]\t\t[ay]\t\t[az]\t\t[pitch]\t\t[yaw]\t\t[roll]\n
    // **************************************************************
    Serial.write((const uint8_t *)&myIMU.ax, sizeof(float));
    Serial.write((const uint8_t *)&myIMU.ay, sizeof(float));
    Serial.write((const uint8_t *)&myIMU.az, sizeof(float));

    Serial.write((const uint8_t *)&myIMU.pitch, sizeof(float));
    Serial.write((const uint8_t *)&myIMU.yaw, sizeof(float));
    Serial.write((const uint8_t *)&myIMU.roll, sizeof(float));

    // GPS
    Serial.write((byte*)&lati,4);
    Serial.write((byte*)&longi,4);
    Serial.println();
    

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 100)
} // void loop()


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}




