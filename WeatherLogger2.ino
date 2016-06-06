/*
 #WeatherLogger
 ----
 Hardware: 
 *LCD screen 20x4 using Adafruit i2c/SPI LCD backpack using MCP23008 I2C expander
 (http://www.ladyada.net/products/i2cspilcdbackpack/index.html)
 *DHT 22 Temperature and Humidity sensor
 *MPL115A2 Pressure sensor
 *Adafruit Data Logger Shield
 
 This sketch shows Date & Time & current sensor readings to LCD
 And logs sensor readings to CVC file on SD card every 5 secconds
 
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
#include <Wire.h>
#include <LiquidTWI.h>
#include <RTClib.h>
#include <DHT.h>
#include <Adafruit_MPL115A2.h>
#include <SD.h>

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)
// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()
#define ECHO_TO_SERIAL   1 // echo data to serial port
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
// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

//Define week days for display
char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
//define mont names for display
char monthsOfTheYear[12][4] ={"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};


//define variables for humidity, temperature C and temperature F
float h = 0.0;
float t = 0.0;
float f = 0.0;

int const AvgSize = 10;
float pressureKPA = 0;
float bpHistory[AvgSize];
float sum;
float bpAvg;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
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
  
  //start up the DHT 22
  dht.begin();
  
  // start up the MP115A2
  mpl115a2.begin();
  
  // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("   Weather Logger");
  //Start up the Clock
  if (! rtc.begin()) {
    lcd.print("Couldn't find RTC");
    while (1);
  }
  //Uncomment the following line to set the date and time of the clock
  //to the compile time of the sketch
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (! rtc.isrunning()) {
    Serial.print("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  //populate Barometric Pressure History
  int i;
  for(i=0;i<AvgSize;i=i+1){
    bpHistory[i] = mpl115a2.getPressure();
    /*Serial.print(i);
    Serial.print(" ");
    Serial.println(bpHistory[i],4);
    */
  }
}

void loop() {
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  //Shift and append Barometric Pressure History
  int i;
  sum = 0;
  for(i=0;i<AvgSize-1;i=i+1){
    bpHistory[i] = bpHistory[i+1];
    sum = sum + bpHistory[i];
    /*Serial.print(i);
    Serial.print(" ");
    Serial.println(bpHistory[i],4);
    */
  }
  bpHistory[AvgSize-1] = mpl115a2.getPressure();
  sum = sum + bpHistory[AvgSize-1];
  pressureKPA = sum/AvgSize;  
  
  UpdateLCD();
  
}

void UpdateLCD(){
  //lcd.clear();
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):

  lcd.setCursor(0, 1);
  DateTime now = rtc.now();
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

  //Output log to serial port
  Serial.print(now.unixtime());
  Serial.print(", ");
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
  Serial.print(", ");
  Serial.print(f);
  Serial.print(", ");
  Serial.print(h);
  Serial.print(", ");
  Serial.println(pressureKPA);
}

