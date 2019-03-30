/*
#WeatherLogger

This sketch shows Date & Time & current sensor readings to LCD and logs sensor readings to CVC file on SD card every 2 secconds

Hardware:
* LCD screen 20x4 using Adafruit i2c/SPI LCD backpack
* DHT 22 Temperature and Humidity sensor
* Adafruit MPL115A2 Pressure sensor breakout
* Adafruit Data Logger Shield


The circuit:
The LCD connected via i2c
* 5V to Arduino 5V pin
* GND to Arduino GND pin
* CLK to Analog #5
* DAT to Analog #4

The DHT 22
* VCC to Arduino 5V pin
* GND to Arduino GND pin
* DAT to Digital #2
* 10K resistor between VCC & DAT as pull up

The MPL115A2 via i2c
* VCC to Arduino 5V pin
* GND to Arduino GND pin
* SCL to Analog #5
* SDA to Analog #4

*/

// Libraries to include:
#include <SD.h>
#include <Wire.h>
#include <LiquidTWI.h>
#include <RTClib.h>
#include <DHT.h>
#include <Adafruit_MPL115A2.h>


// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  2000 // mills between entries (reduce to take more/faster data)
// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 20000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//#Initialize Hardware
// Connect LCD via i2c, default address #0 (A0-A2 not jumpered)
LiquidTWI lcd(0);
// Real Time Clock
RTC_DS1307 rtc;
//DHT 22 sensor
#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
//mp115a2 baramoter sensor
Adafruit_MPL115A2 mpl115a2;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

//Define week days for display
char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
//define mont names for display
char monthsOfTheYear[13][4] ={"   ","Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};


//define variables for humidity and temperature F
float h = 0.0;
float f = 0.0;

//#define AvgSize  10
float pressureKPA = 0;
//float bpHistory[AvgSize];

void error(char *str)
{
  lcd.clear();
  lcd.print("error: ");
  lcd.println(str);
#if ECHO_TO_SERIAL
  Serial.println(str);
#endif  //ECHO_TO_SERIAL
  while(1);
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("   Weather Logger");

  // initialize the SD card
  Serial.print("Initializing SD card...");
  lcd.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  lcd.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
  lcd.print("Logging to: ");
  lcd.println(filename);
  
  //start up the DHT 22
  dht.begin();
  
  // start up the MP115A2
  mpl115a2.begin();
  
  
  //Start up the Clock
  if (! rtc.begin()) {
    error("RTC failed");
    while (1);
  }
  //Uncomment the following line to set the date and time of the clock
  //to the compile time of the sketch
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (! rtc.isrunning()) {
    error("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
  logfile.println("millis,stamp,datetime,humidity,temp,pressure");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,humidity,temp,pressure");
#endif //ECHO_TO_SERIAL
/*  
  //populate Barometric Pressure History Array
  for(int i=0;i<AvgSize;i=i+1){
    bpHistory[i] = mpl115a2.getPressure();
  }
*/
 lcd.clear();
 lcd.print("   Weather Logger");
}

void loop() {
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif

  // fetch the time
  now = rtc.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(f)) {
    error("Failed to read from DHT sensor!");
    return;
  }
 /* 
  //Shift and append Barometric Pressure History
  int sum = 0;
  for(int i=0;i<AvgSize-1;i=i+1){
    bpHistory[i] = bpHistory[i+1];
    sum = sum + bpHistory[i];
  }
  bpHistory[AvgSize-1] = mpl115a2.getPressure();
  sum = sum + bpHistory[AvgSize-1];
  pressureKPA = sum/AvgSize;  
 */
  pressureKPA = mpl115a2.getPressure();
  
  logfile.print(", ");    
  logfile.print(h);
  logfile.print(", ");    
  logfile.print(f);
  logfile.print(", ");    
  logfile.print(pressureKPA);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(h);
  Serial.print(", ");    
  Serial.print(f);
  Serial.print(", ");    
  Serial.print(pressureKPA);
#endif //ECHO_TO_SERIAL
  
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  lcd.setCursor(0, 1);
  char DTStr [20];
  int n;
  n = sprintf(DTStr, "%s %s %02d  %02d:%02d:%02d", daysOfTheWeek[now.dayOfTheWeek()],monthsOfTheYear[now.month()],now.day(),now.hour(),now.minute(),now.second());
  lcd.print(DTStr);

  lcd.setCursor(0,2);
  lcd.print("H:");
  lcd.print(h);
  lcd.print("% Temp:");
  lcd.print(f);
  lcd.print("F");

  lcd.setCursor(0,3);
  lcd.print("Press: "); 
  lcd.print(pressureKPA); 
  lcd.print(" kPa");
  
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();
}
