#include <dht_nonblocking.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

// Define Pins for LEDs
#define BLUE 53 //Turn on when motor is running.
#define GREEN 51 //This LED is lit until the fan is turned on (idle state)
#define RED 47 //turns on if water level too low, all other LEDs off until water level up
#define YELLOW 49 //remains on until system starts (disable state)
//END OF LED pin definitions

#define DHT_SENSOR_TYPE DHT_TYPE_11
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


Servo myservo;
const int A = 0b00000010;

//integer variables for LEDs
int redValue = 255;
int greenValue = 255;
int blueValue = 255;
int yellowValue = 255;

//How to turn LED on/off
// analogWrite(RED, redValue);

//Define ADC Register Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
//end of water sensor and servo register pointers

//variables for water sensor code.
//had to edit these variables to get the water sensor code to work.
int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

static const int DHT_SENSOR_PIN = 2; //pin two for temperature sensor
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//define port E pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

//define PORT B pointers
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

//define PORT L pointers
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;


//define port G pointers
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;
//end of DC motor code


int inPin = 52;         // the number of the input pin
int outPin = 50;       // the number of the output pin

int state = LOW;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = HIGH;    // the previous reading from the input pin

long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

void setup( )
{
  Serial.begin( 9600);
  adc_init(); //initialized water and servo controls
  myservo.attach(13);
  myservo.write(90);// move servos to center position -> 90°
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //pinMode(RED, OUTPUT);
  *ddr_l |= B00000100;
  //pinMode(GREEN, OUTPUT);
  *ddr_b |= B00000100;
  //pinMode(BLUE, OUTPUT);
  *ddr_b |= B00000001;
  //pinMode(YELLOW, OUTPUT);
  *ddr_l |= B00000001;
  //digitalWrite(RED, LOW);
  *ddr_l &= B11111011;
  //digitalWrite(GREEN, LOW);
  //digitalWrite(BLUE, LOW);
  
  //digitalWrite(YELLOW, HIGH);
  *port_l |= B00000001;

 //DC motor setup
 //Set PE5 to output
  *ddr_e |= 0x20;
//set PE3 to output
  *ddr_e |= 0x08;
//set PG5 to output
  *ddr_g |= 0x20;

  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);

  //error messages for RTC
 if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
 }
 if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
 }
}

void loop( )
{  
  float temperature;
  float humidity;
  reading = digitalRead(inPin);

  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH){
      state = LOW;
      Serial.print("OFF\n");
    }
    else{
      Serial.print("ON\n");
      state = HIGH;
    }
    time = millis();    
  }

  digitalWrite(outPin, state);

  previous = reading;

  if(state == HIGH){
  if( measure_environment( &temperature, &humidity ) == true )
  {
    timeStamp();
    Serial.print( "Temperature: " );
    float op1 = temperature * 1.8;
    float op2 = op1 + 32; //conversion from C to F
    temperature = op2;
    Serial.print(temperature, 1);
    Serial.print("\n");
    Serial.print( "deg. F, Humidity = " );
    Serial.print(humidity, 1);
    Serial.print("%\n");
  }
    //water sensor loop code:
    int value = adc_read(adc_id); // get adc value
    
    //test purposes:
   // int value = 151; // get adc value

    //red LED water level low loop
    if(value < 50){ errorLED(value); }
    
    if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10)))
    {
      sprintf(printBuffer,"Water level is %d\n", value);
      Serial.print(printBuffer);
      HistoryValue = value;
    } //end of water sensor loop code

    //servo loops:
  servoLoop();
  
if(temperature > 0){
  lcdScreen(temperature, humidity);
}

  motorToggle(temperature, value);
  }
  else if(state == LOW){
  lcd.setCursor(0, 0);
  lcd.print("STATUS:           ");
  lcd.setCursor(0, 1);
  lcd.print("IDLE...          ");

  //analogWrite(BLUE, 0);
  *ddr_b |= B00000001;
  //analogWrite(RED, 0);
   *ddr_l &= B11111011;
  //analogWrite(YELLOW, yellowValue);
  *port_l |= B00000001;
   //analogWrite(GREEN, 0);
  *port_b &= B11111011;
    
  }
}

void motorToggle(float temperature, float value){
      if(temperature > 76 && value > 50){
  //write a 1 to the enable bit on PE3
  *port_e |= 0x08;
  //analogWrite(BLUE, blueValue);
  *port_b |= B00000001;
  //analogWrite(RED, 0); && analogWrite(YELLOW, 0);
  *port_l &= 11111010;
   //analogWrite(GREEN, 0);
  *port_b &= B11111011;
  }
  
  if(temperature < 76){
  *port_e &= 0x00;
  //analogWrite(BLUE, 0);
  *port_b &= B11111110;
  //analogWrite(RED, 0);
  //analogWrite(YELLOW, 0);
  *port_l &= B11111010;
  //analogWrite(GREEN, greenValue);
  *port_b |= B00000100; 
  }
//write a 1 to PE5
*port_e |= 0x20;

//write a 0 to PG5
*port_g &= 0x20;

  }

void errorLED(int waterLevel){
  //analogWrite(BLUE, 0);
  *port_b &= B11111010;
  //analogWrite(RED, redValue);
  *port_l |= B00000100;
  //analogWrite(YELLOW, 0);
  //analogWrite(GREEN, 0);  
  lcd.setCursor(0, 0);
  lcd.print("Error!         ");
  lcd.setCursor(0, 1);
  lcd.print("WATER TOO LOW!      ");
  waterLevel = adc_read(adc_id);
  *port_e &= 0x00;
  delay(4000);
  if(waterLevel < 50){
  errorLED(waterLevel);
  }
}

void lcdScreen(float temperature, float humidity){
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(humidity, 1);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print("Fahrenheit: ");
  lcd.print(temperature, 1);
}

void servoLoop(){
  int voltage = adc_read(A);//read voltage from POT
  int angle = voltage/5.7;//Scale down analog input to be between 180 and 0
  myservo.write(angle);// move servos   
}

void timeStamp(){
//loop to log time for temperature readings
 DateTime now = rtc.now();
 Serial.print("\nTime: ");
 Serial.print(now.month(), DEC);
 Serial.print('/');
 Serial.print(now.day(), DEC);
 Serial.print(" ");
 int hour = now.hour();
 hour -= 4; //had to adjust the hour because it was NOT reading the correct time.
 Serial.print(hour, DEC);
 Serial.print(':');
 Serial.print(now.minute(), DEC);
 Serial.print(':');
 Serial.print(now.second(), DEC);
 Serial.println();
 delay(3000); 
}

static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
//end of temperature sensor code

//function for water sensor loop and servo loop
void adc_init()
{

  //MUST UPDATE FUNCTION AND SEPERATE THE SERVO AND WATER SENSOR POINTERS.
  // setup the A register
  *my_ADCSRA |= B10000000;
  *my_ADCSRA &= B11110111;
  *my_ADCSRA &= B11011111;
  
  // setup the B register
  *my_ADCSRB &= B11111000;

  // setup the MUX Register
  *my_ADMUX |= (1<<REFS0);
}

//second funciton for water sensor and servo loop:
unsigned int adc_read(unsigned int adc_channel_num)
{
    //MUST UPDATE FUNCTION AND SEPERATE THE SERVO AND WATER SENSOR POINTERS.
  int channel_selector;
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= B11100000;

  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= B11110111;

  //Assign correct channel using MUX 5:0
  if (adc_channel_num < 8) {
    *my_ADMUX |= adc_channel_num;
  }
  else if ((adc_channel_num > 7) && (adc_channel_num < 16)) {
     channel_selector = (adc_channel_num - 8);
     *my_ADCSRB |= B00001000;
     *my_ADMUX |= channel_selector;
  }

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= B01000000;
  
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return (*my_ADC_DATA & 0x03FF);
}