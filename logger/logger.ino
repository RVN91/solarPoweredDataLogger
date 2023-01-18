/*
   Author: Rasmus Nielsen, Ph.D. Fellow at Aalborg University
   
   Complete sketch for a wireless remote data logger. Logs
   sensor input every <logInt> minutes and uploads data log
   every <upInt> minutes.

   General notes:
   1. Accessing data outside of an array can cause AVR reset!
*/

//Arduino and AVR libraries
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SD.h>
#include <SPI.h>

//SODAQ Mbili libraries
#include <RTCTimer.h>
#include <Sodaq_DS3231.h>
#include <GPRSbee.h>
#include <Sodaq_PcInt.h>

//Package with several statistics, such as meadian
#include "QuickStats.h"
QuickStats stats; //Initialize an instance of this class

//Define pins and battery related settings
#define ADC_AREF     3.3 //Reference voltage
#define BATVOLTPIN   A6  //Voltage pin
#define BATVOLT_R1   4.7 //Resistor in battery circuit [ohm]
#define BATVOLT_R2   10  //Resistor in battery circuit [ohm]
#define SD_SS_PIN    11  //Digital pin 11 is the MicroSD slave select pin on the Mbili
#define RTC_PIN      A7  //RTC Interrupt pin
//#define powerPin     18  //Power pin
#define pwPin        9   //PWM input pin

//Define log and upload intervals
const int logInt   = 10; //Interval between each measurement [minutes]
const int upTime   = 60; //Interval between each upload [minutes]
int logCounter     = 0;  //Counter for naming log files

//Network constants
#define APN          "internet"
#define APN_USERNAME ""
#define APN_PASSWORD ""

//FTP server½
#define SERVER   ""
#define USERNAME ""
#define PASSWORD ""
#define FTPPATH  ""

/*
  Debouncing - The RTC interrupt is flagged twice in the PCIF register probably
  due to the RTC chip being limited to the closed second. Hence if the run time
  of the main loop is faster than one second, the next interrupt will happen.

  See http://www.gammon.com.au/switches for technical details.

  Debouncing is fixed by checking if the last interrupt happened longer than
  <debounceDelay> seconds ago.
*/
uint32_t lastDebounceTime = 0;  // The last time the output pin was toggled
uint32_t debounceDelay = 10;    // The debounce time in Epoch / unix time [s]

/*
  Setup for initializing sensors, SD card, and initial test of GPRS communication
*/
void setup() {
  setupSensors();                                //Initialise sensors
  setupLogFile();                                //Initialize SD
  //Test reading
  String fileName = "test.txt";
  String dataRec = createDataRecord();           //Create the data record
  testLogData(dataRec, fileName);   
  uploadData(fileName);
  
  //Get the current log number
  File countFile = SD.open("lCount.txt");
  String logCounterStr = countFile.readString(); //Read content of file
  logCounter = logCounterStr.toInt();            //Convert to int
  countFile.close();                             //Close the file
  setupSleep();
  lastDebounceTime = getNow();  //Set the last interrupt time to the current now  
}

/*
    Main loop for running the program
*/
void loop() {
  rtc.enableInterrupts(EveryMinute);      //Schedule the next wake up
  /*
    Check if at least more time than debounceDelay has passed
  */
  if ((getNow() - lastDebounceTime) > debounceDelay) {
    DateTime wakeTime(getNow());          //Get the current time
    /*
      Log a measurement every logInt minutes and upload every upTime hours
    */
    if (wakeTime.minute() % logInt == 0) {
      String fileName = getFileName(logCounter);
      String dataRec = createDataRecord();                 //Create the data record
      logData(dataRec, fileName);                          //Save the data record to the log file
    }
    
    if (wakeTime.minute() % upTime == 0) {
      String fileName = getFileName(logCounter);
      uploadData(fileName);
      logCounter++; //Update log counter
      File countFile = SD.open("lCount.txt", O_WRITE | O_TRUNC);
      countFile.println(String(logCounter));                //Write the log count
      countFile.close();
      if ((logCounter - 1) % 24 == 0) { //Minus 1 due to the placement of logCounter++
        watchdogSetup();                              //Reset the AVR
      }  
    }
  }


  lastDebounceTime = getNow();  //Set the last interrupt time to the current now
  systemSleep();                //Put system to sleep
}

/*
    Initializing serial port (if debug is enabled), RTC, and PWM for sensor readings
*/
void setupSensors() {
  rtc.begin();               //Initialize the DS3231 RTC
  pinMode(pwPin, INPUT);     //Initialize pins for pulse width input
}

/*
  Generate file name
*/
String getFileName(uint16_t logCounter) {
  String fileName = String(logCounter) + ".txt";
  return fileName;
}

/*
  Set the sleep mode and initialize and wake the
  system at 12:00:00
*/
void setupSleep() {
  pinMode(RTC_PIN, INPUT_PULLUP);           //Set the RTC_PIN to input pullup
  PcInt::attachInterrupt(RTC_PIN, wakeISR); //Attach an interrupt to RTC_PIN
  rtc.begin();                              //Setup the RTC in interrupt mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);      //Set the sleep mode
}

/*
  Initializing the SD card
*/
void setupLogFile()
{
  if (!SD.begin(SD_SS_PIN))
  {
    while (true);
  }
}

/*
  Puts the system in sleep mode
*/
void systemSleep() {
  PcInt::attachInterrupt(RTC_PIN, wakeISR);
  rtc.clearINTStatus(); //Clear interrupt register
  ADCSRA &= ~_BV(ADEN); //Disable ADC
  noInterrupts();       //Disables interrupts
  sleep_enable();       //Enable sleep 
  interrupts();         //Re-enable interrupts
  sleep_cpu();          //Enter sleep mode selected (set the SE (Sleep Enabled) bit)
  sleep_disable();      //Clear the SE bit
  detachInterrupt(RTC_PIN);
  ADCSRA |= _BV(ADEN);  //Enable ADC
}

/*
    Returns the battery voltage [mV]
*/
float getRealBatteryVoltage()
{
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);
  return ((ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage) * 1000;
}

/*
    Returns the median of 10 distance measurements [mm]
    Sensor warmup period of 1 second
*/
float readSensor() {
  pinMode(GROVEPWR, OUTPUT);    //Initialise the GROVEPWR pin to OUTPUT mode (Switched GROOVE sockets)
  digitalWrite(GROVEPWR, HIGH); //Turn on the sensor
  delay(1000);                  //Warm-up period [ms]

  //Measure distance
  float sensorValue1 = pulseIn(pwPin, HIGH);  float sensorValue2 = pulseIn(pwPin, HIGH);
  float sensorValue3 = pulseIn(pwPin, HIGH);  float sensorValue4 = pulseIn(pwPin, HIGH);
  float sensorValue5 = pulseIn(pwPin, HIGH);  float sensorValue6 = pulseIn(pwPin, HIGH);
  float sensorValue7 = pulseIn(pwPin, HIGH);  float sensorValue8 = pulseIn(pwPin, HIGH);
  float sensorValue9 = pulseIn(pwPin, HIGH);  float sensorValue10 = pulseIn(pwPin, HIGH);
  float measurements[10] = {sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8, sensorValue9, sensorValue10};
  int numOfValues = 10;                                   //Number of sensor readings
  float median = stats.median(measurements, numOfValues); //Median filter
  digitalWrite(GROVEPWR, LOW);                            //Turn off the sensor
  pinMode(GROVEPWR, INPUT);                               //Set power pin as input. Conserves more power?
  return median;
}

/*
  Create a String type data record in csv format
  Time stamp, Battery voltage, Distance, Temperature
*/
String createDataRecord()
{
  Serial.begin(9600);

  ///Read the temperature
  rtc.convertTemperature();
  float temp = rtc.getTemperature();

  // Convert temperature voltage to string
  char buffer[14];                                   //make buffer large enough for 7 digits
  String temperatureS = dtostrf(temp, 7, 2, buffer); //'7' digits including '-' negative, decimal and white space. '2' decimal places
  temperatureS.trim();                               //trim whitespace

  //Create a String type data record in csv format
  //TimeDate, Battery Voltage, Distance, Temperature
  String data = getDateTime() + ",";
  data += String(getRealBatteryVoltage()) + ",";
  data += readSensor();
  data += "," + temperatureS;
  return data;
}

/*
  Returns the current date as a string
*/
String getDateTime() {
  String dateTimeStr;
  DateTime dt(rtc.makeDateTime(rtc.now().getEpoch()));  //Create a DateTime object from the current time
  dt.addToString(dateTimeStr);
  return dateTimeStr;
}

/*
  Creates a log file on the SD card and saves the data
*/
void logData(String rec, String fileName)
{
  bool oldFile = SD.exists(fileName);            //Check if the file already exists
  File logFile = SD.open(fileName, O_READ | O_WRITE | O_CREAT | O_APPEND);  //Open the file in write mode
  logFile.println(rec);                          //Write the CSV data
  logFile.close();                               //Close the file to save it
}

/*
  Creates a log file on the SD card and saves the data
*/
void testLogData(String rec, String fileName)
{
  bool oldFile = SD.exists(fileName);            //Check if the file already exists
  File logFile = SD.open(fileName, O_WRITE | O_TRUNC);  //Open the file in write mode
  logFile.println(rec);                          //Write the CSV data
  logFile.close();                               //Close the file to save it
}

/*
  Uploads the data to an FTP server
*/
void uploadData(String fileNameStr)
{
  Serial1.begin(9600);  //Start Serial1 the Bee port
  gprsbee.init(Serial1, BEECTS, BEEDTR);  //Intialise the GPRSbee

  //uncomment this line to debug the GPRSbee with the serial monitor
  //gprsbee.setDiag(Serial);
  
  gprsbee.setPowerSwitchedOnOff(true);  //This is required for the Switched Power method

  if (!gprsbee.openFTP(APN, SERVER, USERNAME, PASSWORD)) {
    return;
  }

  //Create a char with the length of the file name and extension plus 1 to clear space for the null character
  char fileName[fileNameStr.length() + 1];
  //Copy the String into the char
  strcpy(fileName, fileNameStr.c_str());

  //Open / create a file at the FTP server
  if (!gprsbee.openFTPfile(fileName, FTPPATH)) {
    gprsbee.off();
  }

  //Open the file in write mode
  File logFile = SD.open(fileName);

  //Send a complete file to the FTP server
  String P = logFile.readString();
  char O[P.length() + 1];
  P.toCharArray(O, P.length() + 1); //Automatically adds NULL character

  /*
   *  DEBUG MODE: Delete previously generated file
   */
  //SD.remove(fileName);

  //Send data char to FTP server
  if (!gprsbee.sendFTPdata((uint8_t *)O, strlen(O))) {
    // Failed to upload the data
    gprsbee.off();
    return;
  }

  if (!gprsbee.closeFTPfile()) {
    // Failed to close file. The file upload may still have succeeded.
    gprsbee.off();
    return;
  }

  if (!gprsbee.closeFTP()) {
    // Failed to close the connection. The file upload may still have succeeded.
    gprsbee.off();
    return;
  }
}

/*
  Watchdog timer to prevent weird logs by resetting the AVR

  INFO:
  The data logs becomes curropted when the number of logs increases above ~ 140 - 144.
  This is probably due to buffer overflow or bad memory management (bugs / errors in the code). 
  However, it is easier to reset the AVR than to debug the code. 
*/
void watchdogSetup() {
  cli(); //Disables all interrupts on the microcontroller so that configuration is never disrupted and left unfinished
  wdt_reset(); //Resets the watchdog timer

  /*   
    WDTCSR configuration:   
    WDIE  = 1: Interrupt Enable   
    WDE   = 1 :Reset Enable   

    See table for time-out variations:   
    WDP3 = 0 :For 1000ms Time-out   
    WDP2 = 1 :For 1000ms Time-out   
    WDP1 = 1 :For 1000ms Time-out   
    WDP0 = 0 :For 1000ms Time-out
  */

  // Enter Watchdog Configuration mode (enables configuration of the register)
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set Watchdog settings: 
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3)  | (1<<WDP2) | (1<<WDP1)  | (0<<WDP0);

  sei(); //Enable intterrupts
  delay(2000);  //Add + 1 second to be sure that WDT resets the AVR!
  wdt_reset(); //Resets the watchdog timer
}

/*
    Interrupt Service Routine: Keep it empty!
*/
void wakeISR() {
}

/*
  Gets the current time in EPOCH / UNIX time
*/
uint32_t getNow() {
  return rtc.now().getEpoch();
}
