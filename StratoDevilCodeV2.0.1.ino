

//**********Libraries************************
#include <Wire.h> // Library to use I2C communication
#include<SPI.h>   // SPI library
#include<SD.h>    // SD Card Library
#include "Adafruit_MPRLS.h" // Library for Pressure Sensor
#include <ErriezDHT22.h>     //Library for Humidity Sensor
#include <Arduino.h>            //Library for VOC
#include <SensirionI2CSen5x.h>   //Library for VOC

//**********SD CARD DECLARATIONS ****************
String file_name = "Data.csv"; // File name must follow 8.3 rule. <= 8 characters b4 period. only 3 characters for extension. "12345678.123" <-- Max characters. https://www.arduino.cc/en/Reference/SDCardNotes
File ascendDataObj;            // Create a File object, that allows for reading and writing data.

//**********BATTERY DECLARATIONS*****
int battPin = A6;
float voltage;

//**********Accelerometer declarations********
long accelX, accelY, accelZ; // The raw value from the sensor. Must be converted later in excel.

//***********Humidity New Declarations**********
#define HYT_ADDR 0x28 //I2C address
float HUMIDITY;
float HTEMP;

//***********Humidity Old Declarations**********
#define DHT22_PIN      9 //Pin Number
DHT22 dht22 = DHT22(DHT22_PIN); // Create DHT22 sensor object
double humidityVal;
double humTemp;

//***********Pressure Declarations**********
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
float Pressure_data;

//***********Temperature Declarations**********
int inTmpPin  = A0; // Pin that was set PCB
int extTmpPin = A1; // Pin that was set PCB
float inTmpV;      // Converts reading into a voltage
float extTmpV;     // Converts reading into a voltage
float inTmpC;      // Converts reading into a voltage
float extTmpC;    // Converts reading into a voltage

//***********VOC Declarations**********
SensirionI2CSen5x sen5x;

float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

//***********Geiger Declarations**********
float cp2s; //Geiger value

//*********OTHER DECLARATIONS***********************
int t_rdData; // Time counter for when reading data
const int LED_pin = 2; // LED pin I used in Eagle Design.
int SDpin=10;
void setup() {
//**********Beginning Initializations**********
  Serial.begin(115200);       // Initialize Serial Monitor
  pinMode(LED_pin, OUTPUT); // LED_pin is an indicator
  Wire.begin();             // Initialize Wire library for I2C communication
  pinMode(10,OUTPUT);
  SD.begin();
  setupMPU();               // Initialize the MPU (the accelerometer)
  requestH();               // Initialize New Humidity Sensor
  mpr.begin();              // Initialize Pressure Sensor
  sen5x.begin(Wire);        //VOC
  sen5x.startMeasurement(); //VOC
  delay(3000);
  
//**********Geiger INITIALIZATION***********
cp2s = 0;
attachInterrupt(3, tube_impulse, RISING);   //checks INT.0 which is actually digital pin 'D2' (calls for [tube_impulse] whenever the microcontroller detects a rise in voltage)

//**********Accelerometer INITIALIZATION***********
accelX = 0;
accelY = 0;
accelZ = 0;

//**********SD CARD INITIALIZATION***********
  while(!SD.begin()) { // While SD cannot initialize, turn LED on
    Serial.print("error");
    digitalWrite(LED_pin, HIGH);
  }
  
  ascendDataObj = SD.open(file_name, FILE_WRITE);                         // Opens file to write to
  ascendDataObj.print(F("t,Volt,aX,aY,aZ,H,Htemp,Pm1p0,Pm2p5,Pm4p0,Pm10p0,VOCRH,VOCT,VOC,TIn,TOut,P,Geiger")); // Data MUST be written in this order to make sense
  ascendDataObj.println();                                               // Move to the next line before reading data
  ascendDataObj.close();                                                // Save file

// Blink LED 3 times to indicate succesful intialization and file creation
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_pin, HIGH);
    delay(500);
    digitalWrite(LED_pin, LOW);
    delay(500);
  }

delay(5000); // give Mega 5s before reading data
}

void loop() {

  //**********PREPARE SD CARD**********
  t_rdData = t_rdData + 3; //Increment t
  ascendDataObj = SD.open(file_name, FILE_WRITE);         // Prepare to write data by OPENING the data file, This acts like a Serial monitor now. ascendDataObj.print() and ascendDataObj.println()
  
  //**********Get Sensor Readings**********

  // BATTERY VOLTAGE
  voltage = analogRead(battPin) * (5.0 / 1023.0); // Analog reading goes from 0 - 1023

  // Accelerations
  recordAccelRegisters(); // Get Acceloremeter Values

  // Humidity New - DGL
  requestH();
  
  // VOC - DGL
  sen5x.readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);
  
  // Temperature
  inTmpV = analogRead(inTmpPin) * (5.0 / 1024.0); // For memory purposes it might be best to just get the analog reading and convert it into Excel later
  extTmpV = analogRead(extTmpPin) * (5.0 / 1024.0);
  inTmpC = (inTmpV - 0.5) * 100;
  extTmpC = (extTmpV - 0.5) * 100;

  // Pressure
  Pressure_data = mpr.readPressure();

  // Geiger
      if (cp2s > 255)   {
    cp2s = 255;  }

  //**********Debugging SerialPrints - Will be deleted later or commented out for flight****************
  Serial.println(F("t,Volt,aX,aY,aZ,H,Htemp,Pm1p0,Pm2p5,Pm4p0,Pm10p0,VOCRH,VOCT,VOC,TIn,TOut,P,Geiger")); // Data MUST be written in this order to make sense  
  Serial.print(t_rdData); Serial.println(", Time"); 
  Serial.print(voltage); Serial.println(", Voltage"); 
  Serial.print(accelX); Serial.println(", Ax");
  Serial.print(accelY); Serial.println(", Ay");
  Serial.print(accelZ); Serial.println(", Az");
  Serial.print(HUMIDITY);Serial.println(", New Humidity");
  Serial.print(HTEMP);Serial.println(", New H Temp"); 
  Serial.print("MassConcentrationPm1p0:"); Serial.print(massConcentrationPm1p0); Serial.print("\t");
  Serial.print("MassConcentrationPm2p5:"); Serial.print(massConcentrationPm2p5); Serial.print("\t");
  Serial.print("MassConcentrationPm4p0:"); Serial.print(massConcentrationPm4p0); Serial.print("\t");
  Serial.print("MassConcentrationPm10p0:"); Serial.print(massConcentrationPm10p0); Serial.print("\t");
  Serial.print("AmbientHumidity:"); Serial.print(ambientHumidity); Serial.print("\t");
  Serial.print("AmbientTemperature:"); Serial.print(ambientTemperature); Serial.print("\t");
  Serial.print("VocIndex:"); Serial.print(vocIndex); Serial.println();
  Serial.print(inTmpC); Serial.println(", Int Temp");
  Serial.print(extTmpC); Serial.println(", Ext Temp");
  Serial.print(Pressure_data); Serial.println(", Pressure");
//  Serial.print(cp2s); Serial.println(", Geiger");

    // Print each value then add a comma. Make sure to print in the same order as the header when we initialized the SD card
  // ascendDataObj.print(F("t,Volt,aX,aY,aZ,H,Htemp,Pm1p0,Pm2p5,Pm4p0,Pm10p0,VOCRH,VOCT,VOC,TIn,TOut,P,Geiger")); // Copied and Pasted from setup to ensure printing in the same data

  ascendDataObj.print(t_rdData); ascendDataObj.print(F(","));       // Print time values
  ascendDataObj.print(voltage); ascendDataObj.print(F(","));        // Print Voltage
  ascendDataObj.print(accelX); ascendDataObj.print(F(","));         // Print Accel X
  ascendDataObj.print(accelY); ascendDataObj.print(F(","));         // Print Accel Y
  ascendDataObj.print(accelZ); ascendDataObj.print(F(","));         // Print Accel Z
  ascendDataObj.print(HUMIDITY); ascendDataObj.print(F(","));       // Print Relative Humidity NEW
  ascendDataObj.print(HTEMP); ascendDataObj.print(F(","));          // Print Temperature from NEW Humidity Sensor
  ascendDataObj.print(massConcentrationPm1p0); ascendDataObj.print(F(","));      // Print Pm1p0
  ascendDataObj.print(massConcentrationPm2p5); ascendDataObj.print(F(","));      // Print Pm2p5
  ascendDataObj.print(massConcentrationPm4p0); ascendDataObj.print(F(","));      // Print Pm4p0
  ascendDataObj.print(massConcentrationPm10p0); ascendDataObj.print(F(","));     // Print Pm10p0
  ascendDataObj.print(ambientHumidity); ascendDataObj.print(F(","));             // Print VOC RH
  ascendDataObj.print(ambientTemperature); ascendDataObj.print(F(","));          // Print VOC Temp
  ascendDataObj.print(vocIndex); ascendDataObj.print(F(","));                    // Print tVOC
  ascendDataObj.print(inTmpC); ascendDataObj.print(F(","));         // Print Temp1
  ascendDataObj.print(extTmpC); ascendDataObj.print(F(","));        // Print Temp2
  ascendDataObj.print(Pressure_data); ascendDataObj.print(F(","));  // Print Pressure
  ascendDataObj.print(cp2s); ascendDataObj.print(F(","));           // Print Geiger  
  ascendDataObj.println();                                          // Create a new line b4 getting and printing next readings from all sensors
  ascendDataObj.close();                                            // Closes/Saves data to file.

    cp2s = 0; //resets the Geiger count
  //********** FLASH LED*********
  digitalWrite(LED_pin,LOW);
  delay(3000);                // Humidity Sesnor needs atleast 2 seconds before a new reading will become available - DGL
  digitalWrite(LED_pin,HIGH); // Flashes LED

  //**********MAKE SURE SD CARD IS STILL INTACT**************
  checkSDduringFlight();
  
}

//********************Methods used in loop*****************

void checkSDduringFlight() {
  int t_interrupt = 0; //Initialize count for how long it was interrupted
  if (!SD.begin()) {
    while (!SD.begin()) {
      delay(1000); //Try to connect every 1s
      t_interrupt++; //Increment the interrupted count
      t_rdData++;           //Can't forget to increment the time for recording data
      digitalWrite(LED_pin, HIGH); //Solid red to say SD card is not able to receive data
    }
    digitalWrite(LED_pin, LOW); //Turn LED off once it's able to write to again
    ascendDataObj = SD.open(file_name, FILE_WRITE); // Opens the file
    ascendDataObj.print(F("Data was interrupted for "));
    ascendDataObj.print(t_interrupt); ascendDataObj.println(F(" s")); // Writes for how long
    ascendDataObj.println(); // Make a new line
    ascendDataObj.close(); // Saves the file
  }
}

void setupMPU() {
  // Omitted the setup for the Gyroscope since knowing the orientation is not necessary.
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00010000); //Setting the accel to +/- 8g's
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
   (Wire.available() < 6); 
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  // To get gForce divide accelerations by 4096 (epends on max readings. Refer to datasheet).
}

void requestH(){
        Wire.beginTransmission(HYT_ADDR);   // Begin transmission with given device on I2C bus
        Wire.requestFrom(HYT_ADDR, 4);
        if(Wire.available() == 4) {                   
            int b1 = Wire.read();
            int b2 = Wire.read();
            int b3 = Wire.read();
            int b4 = Wire.read();
            Wire.endTransmission(); 
            int rawHumidity = b1 << 8 | b2;
            rawHumidity = (rawHumidity &= 0x3FFF);
            float humidity = 100.0 / pow(2,14) * rawHumidity;
            b4 = (b4 >> 2); // Mask away 2 least significant bits see HYT 221 doc
            int rawTemperature = b3 << 6 | b4;
            float temperature = 165.0 / pow(2,14) * rawTemperature - 40;
            HUMIDITY = humidity;
            HTEMP = temperature;    
        }
        else {
         HUMIDITY = 0;
         HTEMP = 0;  
        }
 }  

void tube_impulse(){ //subprocedure for capturing events from Geiger Kit
cp2s++;
}
