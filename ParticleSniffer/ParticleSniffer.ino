#include <TinyGPS.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>
#include <SD.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  p a r t i c l e   s n i f f e r
//
// 5/18/2013 - ParticleSniffer - prototype release candidate 1
// 
// 4/14/2013 - SnifferDev01 - GPS parsing functionality
// 4/28/2013 - SnifferDev02 - Sharp GP2Y1010AU0F optical dust sensor added
// 5/05/2013 - SnifferDev03 - SD card writer added
// 5/08/2013 - SnifferDev04 - Low Battery Testing
// 5/08/2013 - SnifferDev05 - adafruit gps sd shield
// 5/08/2013 - SnifferDev06 - getting ready for transfer
// 5/18/2013 - SnifferDev07 - added support for 4 instruments
// 5/19/2013 - SnifferDev08 - incorporated Melissa's 19 May email
// 
// Compatible with the Arduino MEGA 2560
// The Uno doesn't work as there is no pin change interrupts available.
//
// Sniffer ID number regions:
// 
//     0             - unused
//     1 through  19 - testing units
//                      * 001 is vince
//                      * 002 is the unit delivered to Melissa on 13/05/19
//    20 through 119 - Particle Sniffer production units
//   120 through 139 - Rooster tails testing units
//   140 through 255 - Rooster Tails production units
//
// The GPS parsing code is from http://arduiniana.org/libraries/tinygps/
// and was written by Mikal Hart; last updated May 30, 2012
//
// GPS data comes in from RX1 on the Arduino MEGA (pin 19)
// the Sharp dust sensor needs a connection to its LED on pin 12, and
// a connection to an analog input (A6) for the sensor.
// 
// Sensor documentation:
//
// Sharp Optical Dust Sensor GP2Y1010AU0F
// 
// The Sharp dust sensor code is from Trefex (Christophe Trefois) and 
// SerialC (Cyrille Médard de Chardon), who are located in Luxembourg.
// see http://arduinodev.woofex.net/2012/12/01/standalone-sharp-dust-sensor/
//
// The linear eqaution is taken from http://www.howmuchsnow.com/arduino/airquality/
// Chris Nafis (c) 2012
//
// The datasheet for the sensor is here: https://www.sparkfun.com/datasheets/Sensors/gp2y1010au_e.pdf
//
// Sharp pin 1 (V-LED)   => connected to 150ohm resister to Vcc and + side of cap with - to Arduino GND
// Sharp pin 2 (LED-GND) => Arduino GND
// Sharp pin 3 (LED)     => Arduino pin 6
// Sharp pin 4 (S-GND)   => Arduino GND
// Sharp pin 5 (Vo)      => Arduino Analog 4 pin
// Sharp pin 6 (Vcc)     => Arduino 5V
// 
// The Sharp Arduino sensor connector pinout is here:
// 
// 1  Black
// 2  Gray
// 3  Brown
// 4  black
// 5  Purple
// 6  White
// 7  NC
// 8  plugged
// 
// License:
// --------
// This is free software. You can redistribute it and/or modify it under
// the terms of Creative Commons Attribution 3.0 United States License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
// or send a letter to Creative Commons, 171 Second Street, Suite 300, 
// San Francisco, California, 94105, USA.
// 
////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEBUG

#ifdef DEBUG
#define DPRINT Serial.print
#define DPRINTLN Serial.println
#define DPRINT2 Serial.print
#define DPRINTLN2 Serial.println
#else
#define DPRINT(x)
#define DPRINTLN(x)
#define DPRINT2(x,y)
#define DPRINTLN2(x,y)
#endif 

#define CSV    /* define if CSV instead of space-delimited text desired; .TXT changes to .CSV */

#define APP_TITLE_TEXT  "\nSnifferDev08"


////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  pins and constants
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// digital arduino mega pins used

#define ARD_AVAILABLE_2              2   /* unassigned */
#define ARD_AVAILABLE_3              3   /* unassigned */
#define ARD_AVAILABLE_4              4   /* unassigned */
#define ARD_AVAILABLE_5              5   /* unassigned */
#define SHARP_LED_PIN                6   /* pin to control Sharp LED */
#define ARD_SS_TX                    7   /* unassigned */
#define ARD_SS_RX                    8   /* unassigned */
#define SD_CD_PIN                    9   /* pin to do chip detect for SD card reader/writer */
#define SD_CCS_PIN                  10   /* pin to do chip select for SD card reader/writer */
#define SD_MOSI_PIN                 11   /* MOSI pin for SD card reader/writer */
#define SD_MISO_PIN                 12   /* MISO pin for SD card reader/writer */
#define SD_SCK_PIN                  13   /* SCK pin for SD card reader/writer */
#define RED_LED                     13   /* red LED on arduino board */
#define GPS_RX                      19   /* RX pin for serial port 1 on the Arduino MEGA */

// analog arduino mega pins used

#define ARD_ANALOG_AVAILABLE_0       0   /* unassigned */
#define ARD_ANALOG_AVAILABLE_1       1   /* unassigned */
#define ARD_ANALOG_AVAILABLE_2       2   /* unassigned */
#define ARD_ANALOG_AVAILABLE_3       3   /* unassigned */
#define SHARP_MEASURE_PIN            4   /* pin to read analog Sharp output */
#define FIVEV_MEASURE_PIN            5   /* pin to read analog 5V output */
#define ARD_ANALOG_AVAILABLE_6       6   /* unassigned */
#define ARD_ANALOG_AVAILABLE_7       7   /* unassigned */
#define ARD_ANALOG_AVAILABLE_8       8   /* unassigned */
#define ARD_ANALOG_AVAILABLE_9       9   /* unassigned */
#define ARD_ANALOG_AVAILABLE_10     10   /* unassigned */
#define ARD_ANALOG_AVAILABLE_11     11   /* unassigned */
#define ARD_ANALOG_AVAILABLE_12     12   /* unassigned */
#define ARD_ANALOG_AVAILABLE_13     13   /* unassigned */
#define ARD_ANALOG_AVAILABLE_14     14   /* unassigned */

//constants

#define EEPROM_sig                   0   /* offset in EEPROM to signature byte */
#define EEPROM_id                    1   /* EEPROM offset to Sniffer ID */

#define EEPROM_SIG_BRANDED          69
#define DEFAULT_SNIFFER_ID           3   /* ID assigned to this sniffer, in dev range, which is up to 99 */
#define BROWNOUT_LEVEL             650   /* if falls below this value, no writing */
#define MS_WAKEUP_TIME_FOR_GPS    2000   /* allow 2 sec of GPS before a reading */

#define MS_BETWEEN_POSTS_TO_LOG  15000  /* how many millisconds between posts */
//#define MS_BETWEEN_POSTS_TO_LOG   2010   /* faster for testing */

#ifdef CSV
#define DELIMITER_CHARACTER      ","   /* character used to separate data items */
#define FILE_SUFFIX           "%03ddata.csv"
#else
#define DELIMITER_CHARACTER      " "
#define FILE_SUFFIX           "%03ddata.txt"
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  structures
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// variables having to do with LEDS
typedef struct _snifferS {
  byte    id;                          // ID number of this sniffer
  char    fileName[20];                // name of our file
  int     raw_brownout_reading;        // value of battery level
  int     raw_co_reading;              // sensitive to incomplete combustion, and I think it may be particularly sensitive to fires
  int     raw_no_reading;              // combustion emission primarily from engines, both gas and diesel
  int     raw_co2_reading;             // sensitive to all combustion in the basic sense that the fuel converts to water and CO2
  int     raw_no2_reading;             // also combustion, a more expensive sensor that it not currently in stock
  int     raw_temp_reading;            // necessary as the performance of the sensors is a function of these variables
  int     raw_rh_reading;              // necessary
  int     raw_particles_reading;       // we have the Sharp sensors, and I am going to order a second type tomorrow
  int     raw_gasdiesel_reading;       // a mixture of CO, NO2, and some hydrocarbons, something I thought I might try out, but not as central.
  float   dust_reading;                // dust density from sharp sensor
  boolean bBrownOut;                   // true if voltage below minimum level
  boolean bDataCard;                   // true if data card inserted
  boolean bBeginDone;                  // true if begin already called
  boolean bFileOpen;                   // true of file open
  boolean bIgnoreGPS;                  // true if ignoring GPS
  boolean b1v1_mode;                   // true if aRef is 1.1v 
} snifferS;
struct _snifferS sniffer;              //  S N I F F E R

TinyGPS gps;
File dataFile;

HardwareSerial gpsSerial = Serial1;

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  setup
//
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);        // for debugging
#endif
  gpsSerial.begin(9600);        // for GPS

  if(EEPROM.read(EEPROM_sig) != EEPROM_SIG_BRANDED)  // retrieve EEPROM data here
  {
    sniffer.id = DEFAULT_SNIFFER_ID;
    EEPROM.write(EEPROM_sig, EEPROM_SIG_BRANDED);
    EEPROM.write(EEPROM_id, sniffer.id);
  }
  else
  {
    sniffer.id = EEPROM.read(EEPROM_id);
  }
  sprintf(sniffer.fileName, FILE_SUFFIX, sniffer.id);  // build the SD card filename
  
  pinMode(RED_LED, OUTPUT);
  // set up Sharp LED pin output
  pinMode(SHARP_LED_PIN, OUTPUT);
  // make sure that the default chip select pin is set to output
  pinMode(SD_CCS_PIN, OUTPUT);
  // enable SD card detect pin
  pinMode(SD_CD_PIN, INPUT_PULLUP);
  
  sniffer.bBrownOut  = false;
  sniffer.bFileOpen  = false;
  sniffer.bDataCard  = false;
  sniffer.bBeginDone = false;
  sniffer.bIgnoreGPS = false;
  sniffer.b1v1_mode  = false;
  
  DPRINTLN(APP_TITLE_TEXT);
  DPRINTLN();
  DPRINTLN("CPU    Bttry Part CO    NO    CO2   NO2   Temp  RH    Part  GasDi Part  Latitude   Longitude  Date       Time     Alt");
  DPRINTLN("Time   Level ID # Raw   Raw   Raw   Raw   Raw   Raw   Raw   Raw   Dens  (deg)      (deg)                          (m)");
  DPRINTLN("------ ----- ---- ----- ----- ----- ----- ----- ----- ----- ----- ----- ---------- ---------- ---------- -------- --------");
  
  FlexiTimer2::set(50, 1.0/1000, ISR_timer);    // run interrupt code 50 times a second
  FlexiTimer2::start();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  interrupt service routines
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// called from a 1 ms interrupt on timer 2
void ISR_timer(void) 
{ 
  feedgps();
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  gpsdump(gps);
  sniffer.bIgnoreGPS = true;
  interruptable_delay(MS_BETWEEN_POSTS_TO_LOG - MS_WAKEUP_TIME_FOR_GPS);
  sniffer.bIgnoreGPS = false;
  interruptable_delay(MS_WAKEUP_TIME_FOR_GPS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// helper functions
//
////////////////////////////////////////////////////////////////////////////////////////////////////

static void feedgps()
{
  char chr;
  
  while (gpsSerial.available())
  {
    chr = gpsSerial.read();
    if (sniffer.bIgnoreGPS == false)
    {
      gps.encode(chr);
    }
  }
}

static void gpsdump(TinyGPS &gps)
{
  float          flat, flon;
  int year;
  byte month, day, hour, minute, second, hundredths;
  
  unsigned long  age, chars = 0;
  unsigned short sentences = 0, failed = 0;
  
  // check if battery voltage dropping below brownout level
  if (sniffer.bBrownOut == false)
    sniffer.raw_brownout_reading = raw_5v_reading();
    if (sniffer.raw_brownout_reading < BROWNOUT_LEVEL)
      sniffer.bBrownOut = true;

  if (sniffer.bBrownOut == true)
  {
    digitalWrite(RED_LED, HIGH);
  }
  else
  {  
    // not brownout; proceed to sample
    sniffer.raw_co_reading = raw_co_reading();
    sniffer.raw_no_reading = raw_no_reading();
    sniffer.raw_co2_reading = raw_co2_reading();
    sniffer.raw_no2_reading = raw_no2_reading();
    sniffer.raw_temp_reading = raw_temp_reading();
    sniffer.raw_rh_reading = raw_rh_reading();
    sniffer.raw_particles_reading = raw_particles_reading();
    sniffer.raw_gasdiesel_reading = raw_gasdiesel_reading();

    sniffer.dust_reading = dust_reading(sniffer.raw_particles_reading);
    // add sample code for sensors 2 through 4 here
    
    // check if gps data valid
    gps.f_get_position(&flat, &flon, &age);
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

    if ((flat != TinyGPS::GPS_INVALID_F_ANGLE) && (month != 0))
    {
      // test if card inserted since last writing
      if (digitalRead(SD_CD_PIN) == HIGH) // high if inserted
      {
  //      DPRINTLN("line is high");
        // if first time or card was reinserted
        if(sniffer.bDataCard == false)
        {
  //        DPRINTLN("card first time or reinserted");
          sniffer.bDataCard = true;
          if (sniffer.bBeginDone == false)
          {
  //          DPRINTLN("card first time");
            sniffer.bBeginDone = true;
            if (!SD.begin(SD_CCS_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN)) 
            {
  //            DPRINTLN("Card failed, or not present");
              sniffer.bBeginDone = false;
              sniffer.bDataCard = false;
            }
          } 
  //        else // if card in, bit first time 
  //        {
  //          DPRINTLN("card re-inserted");
  //        }
        } 
        // card status is inserted
        if (sniffer.bDataCard == true)
        {
          // Open up the file we're going to log to
          sniffer.bFileOpen = true;
          dataFile = SD.open(sniffer.fileName, FILE_WRITE);
          if (! dataFile) {
  //          DPRINTLN("error reopening SD card file");
            sniffer.bFileOpen = false;
          }
        }
      }
      else
      {
  //      DPRINTLN("line is LOW");
        if(sniffer.bDataCard == true)
        {
  //        DPRINTLN("card removed");
          sniffer.bDataCard = false;
        }
      }
      // write info out to sd card and debug port
      print_int(millis()>> 8, TinyGPS::GPS_INVALID_AGE, 6);  // 255 max value
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_brownout_reading, TinyGPS::GPS_INVALID_AGE, 5);  // 1023 max value
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.id, TinyGPS::GPS_INVALID_AGE, 4);  // 255 max value
      log_data(DELIMITER_CHARACTER);

      print_int(sniffer.raw_co_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_no_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_co2_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_no2_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_temp_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_rh_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_particles_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);
      print_int(sniffer.raw_gasdiesel_reading, TinyGPS::GPS_INVALID_AGE, 5);
      log_data(DELIMITER_CHARACTER);

      print_float(sniffer.dust_reading, TinyGPS::GPS_INVALID_F_ANGLE, 5, 2);
      log_data(DELIMITER_CHARACTER);

      print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
      log_data(DELIMITER_CHARACTER);
      print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
      log_data(DELIMITER_CHARACTER);
      print_date(gps);
      print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2);
      DPRINTLN();
    }
    if (sniffer.bFileOpen == true)
    {
      dataFile.println();
      dataFile.close();
      sniffer.bFileOpen = false;
    }
  }
}

static int raw_5v_reading(void)
{
  if (sniffer.b1v1_mode == false)
  {
    sniffer.b1v1_mode = true;
    analogReference(INTERNAL1V1);
    analogRead(FIVEV_MEASURE_PIN);
    analogRead(FIVEV_MEASURE_PIN);
    analogRead(FIVEV_MEASURE_PIN);
    analogRead(FIVEV_MEASURE_PIN);
    delayMicroseconds(90000);          // delta time
  }
  
  int raw = analogRead(FIVEV_MEASURE_PIN); // read the raw signal value (0 to 1023)
 
  delayMicroseconds(40);          // delta time
  return(raw);
}

static int raw_co_reading(void)
{
  return(0);
}

static int raw_no_reading(void)
{
  return(0);
}

static int raw_co2_reading(void)
{
  return(0);
}

static int raw_no2_reading(void)
{
  return(0);
}

static int raw_temp_reading(void)
{
  return(0);
}

static int raw_rh_reading(void)
{
  return(0);
}

static int raw_particles_reading(void)
{
  if (sniffer.b1v1_mode == true)
  {
    sniffer.b1v1_mode = false;
    analogReference(DEFAULT);
    analogRead(SHARP_MEASURE_PIN);
    analogRead(SHARP_MEASURE_PIN);
    analogRead(SHARP_MEASURE_PIN);
    analogRead(SHARP_MEASURE_PIN);
    delayMicroseconds(90000);          // delta time
  }
  
  digitalWrite(SHARP_LED_PIN, LOW); // power on the LED
  delayMicroseconds(280);            // sampling time
 
  int raw = analogRead(SHARP_MEASURE_PIN); // read the raw signal value (0 to 1023)
 
  delayMicroseconds(40);          // delta time
  digitalWrite(SHARP_LED_PIN, HIGH); // turn the LED off
  delayMicroseconds(9680);      // sleep time
  return(raw);
}

static int raw_gasdiesel_reading(void)
{
  return(0);
}


static float dust_reading(int raw)
{
  float voMeasured = raw; // read the raw signal value (0 to 1023)
  // 0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  float calcVoltage = raw * (5.0 / 1024);    // voltage
 
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  float dustDensity = 0.17 * calcVoltage - 0.1;    // dust density
  dustDensity += 0.10;
 
  return dustDensity;
}

static void interruptable_delay(unsigned int iDelay)
{
  unsigned long timeDelay;
  
  timeDelay = millis() + (unsigned long) iDelay;
  while (millis() < timeDelay);
}

static void log_data(char *sz)
{
  if (digitalRead(SD_CD_PIN) == HIGH) // high if inserted
  {
    if (sniffer.bFileOpen == true)
    {
      if(dataFile.print(sz) != strlen(sz))
      {
//        DPRINTLN("write failed");
        // reset sd connection and rewrite
        SD.begin(SD_CCS_PIN, SD_MOSI_PIN, SD_MISO_PIN, SD_SCK_PIN); 
        dataFile.print(sz);
      }
    }
  }
  DPRINT(sz);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  log_data(sz);
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "***********");
    sz[len] = 0;
    for (int i=11; i<len; ++i)
        sz[i] = ' ';
    log_data(sz);
  }
  else
  {
    DPRINT2(val, prec);
    if (sniffer.bFileOpen == true)
      dataFile.print(val, prec);

    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      log_data(" ");
  }
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    log_data("**********,********,");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d", month, day, year);
    log_data(sz);
    log_data(DELIMITER_CHARACTER);
    sprintf(sz, "%02d:%02d:%02d", hour, minute, second);
    log_data(sz);
    log_data(DELIMITER_CHARACTER);
  }
}

//  int16_t crc;
//  if(writeCRC_) {
//    int16_t i, x;
//    // CRC16 code via Scott Dattalo www.dattalo.com
//    for(crc=i=0; i<512; i++) {
//      x   = ((crc >> 8) ^ src[i]) & 0xff;
//      x  ^= x >> 4;
//      crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
//    }
//  } else {
//    crc = 0xffff; // Dummy CRC value
//  }

