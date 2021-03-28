#include <SPI.h>
#include <Ethernet.h>

#include "TimerOne.h"
#include <math.h>

#include "cactus_io_DS18B20.h"
#include "cactus_io_BME280_I2C.h"

#define Bucket_Size 0.01 // bucket size to trigger tip count
#define RG11_Pin 3 // digital pin RG11 connected to
#define TX_Pin 8 // used to indicate web data tx
#define DS18B20_Pin 9 // DS18B20 Signal pin on digital 9

#define WindSensor_Pin (2) // digital pin for wind speed sensor
#define WindVane_Pin (A2) // analog pin for wind direction sensor
#define VaneOffset 0 // define the offset for caclulating wind direction

volatile unsigned long tipCount; // rain bucket tip counter used in interrupt routine
volatile unsigned long contactTime; // timer to manage any rain contact bounce in interrupt routine

volatile unsigned int timerCount; // used to count ticks for 2.5sec timer count
volatile unsigned int timerMinCount; // used to determine one minute count
volatile unsigned long rotations; // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime; // timer to avoid contact bounce in wind speed sensor

long lastTipcount; // keep track of bucket tips
float totalRainfall; // total amount of rainfall detected

volatile float windSpeed;
int vaneValue; // raw analog value from wind vane
int vaneDirection; // translated 0 - 360 wind direction
int calDirection; // calibrated direction after offset applied
int lastDirValue; // last recorded direction value

float minTemp; // keep track of minimum recorded temp
float maxTemp; // keep track of maximum recorded temp

// Create DS18B20, BME280 object
DS18B20 ds(DS18B20_Pin); // on digital pin 9
BME280_I2C bme; // I2C using address 0x77

// Here we setup the web server. We are using a static ip address and a mac address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 45);
EthernetClient client; // create ethernet client

void setup() {

// setup rain sensor values
lastTipcount = 0;
tipCount = 0;
totalRainfall = 0;

// setup anemometer values
lastDirValue = 0;
rotations = 0;

// setup timer values
timerCount = 0;
timerMinCount = 0;

ds.readSensor();
delay(3000); // allow 3 seconds for sensor to settle down
ds.readSensor(); // read again to avoid weird values for defaults
minTemp = ds.getTemperature_C();
maxTemp = ds.getTemperature_C();

// disable the SD card by switching pin 4 High
pinMode(4, OUTPUT);
digitalWrite(4, HIGH);

// start the Ethernet connection and server
Ethernet.begin(mac, ip);

if (!bme.begin()) {
// Serial.println("Could not find BME280 sensor, check wiring!");
while (1);
}

pinMode(TX_Pin, OUTPUT);
pinMode(RG11_Pin, INPUT);
pinMode(WindSensor_Pin, INPUT);
attachInterrupt(digitalPinToInterrupt(RG11_Pin), isr_rg, FALLING);
attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

// setup the timer for 0.5 second
Timer1.initialize(500000);
Timer1.attachInterrupt(isr_timer);

sei();// Enable Interrupts
}

void loop() {

ds.readSensor();
bme.readSensor();

// update rainfall total if required
if(tipCount != lastTipcount) {
cli(); // disable interrupts
lastTipcount = tipCount;
totalRainfall = tipCount * Bucket_Size;
sei(); // enable interrupts
}

// if one minute timer is up then send data to server
if(timerMinCount > 23) {
//reset the timer and rain tip count
cli(); // disable interrupts
timerMinCount = 0;
tipCount = 0;
sei(); // enable interrupts

getWindDirection();

String data = "{\"t\":\"";
data += ds.getTemperature_C();
data += "\",\"h\":\"";
data += bme.getHumidity();
data += "\",\"p\":\"";
data += bme.getPressure_MB();
data += "\",\"ws\":\"";
data += windSpeed;
data += "\",\"wd\":\"";
data += calDirection;
data += "\",\"r\":\"";
data += totalRainfall;
data += "\"}";

digitalWrite(TX_Pin,HIGH); // Turn on red tx led

if (client.connect("data.nevixa.com.au",80)) {
client.println("POST /aws/feed/NVX6023F1 HTTP/1.1");
client.println("Host: data.nevixa.com.au");
client.println("x-feed-id: zzzzzzzzzzzzz");
client.println("x-api-key: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
client.println("content-Type: application/json;charset=utf-8");
client.print("Content-Length: ");
client.println(data.length());
client.println();
client.println(data);
}

delay(1000);
digitalWrite(TX_Pin,LOW); // Turn off red tx led
// stop the connection:
if(client.connected()) {
client.stop(); // Disconnect from the server
}
}
}

// Interrupt handler routine for timer interrupt
void isr_timer() {

timerCount++;

if(timerCount == 5) {
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/2.5) = P * 0.9
windSpeed = rotations * 0.9;
rotations = 0;
timerCount = 0;
timerMinCount++; // increment 1 minute count
}
}

// Interrupt handler routine that is triggered when the rg-11 detects rain
void isr_rg() {

if((millis() - contactTime) > 15 ) { // debounce of sensor signal
tipCount++;
contactTime = millis();
}
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation() {

if((millis() - contactBounceTime) > 15 ) { // debounce the switch contact
rotations++;
contactBounceTime = millis();
}
}

// Get Wind Direction
void getWindDirection() {

vaneValue = analogRead(WindVane_Pin);
vaneDirection = map(vaneValue, 0, 1023, 0, 360);
calDirection = vaneDirection + VaneOffset;

if(calDirection > 360)
calDirection = calDirection - 360;

if(calDirection > 360)
calDirection = calDirection - 360;
}
