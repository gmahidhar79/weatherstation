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
EthernetServer server(80); // create a server listing on 192.168.1.45 port 80

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

ds.readSensor();
minTemp = ds.getTemperature_C();
maxTemp = ds.getTemperature_C();

// disable the SD card by switching pin 4 High
pinMode(4, OUTPUT);
digitalWrite(4, HIGH);

// start the Ethernet connection and server
Ethernet.begin(mac, ip);
server.begin();

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

// update min and max temp values
if(ds.getTemperature_C() < minTemp) {
minTemp = ds.getTemperature_C();
}
if(ds.getTemperature_C() > maxTemp) {
maxTemp = ds.getTemperature_C();
}

// update rainfall total if required
if(tipCount != lastTipcount) {
cli(); // disable interrupts
lastTipcount = tipCount;
totalRainfall = tipCount * Bucket_Size;
sei(); // enable interrupts
}

// listen for incoming clients
EthernetClient client = server.available();
if (client) {
// an http request ends with a blank line
boolean currentLineIsBlank = true;
while (client.connected()) {
if (client.available()) {
char c = client.read();
Serial.write(c);
// if you've gotten to the end of the line (received a newline
// character) and the line is blank, the http request has ended,
// so you can send a reply
if (c == '\n' && currentLineIsBlank) {
// send a standard http response header
digitalWrite(TX_Pin,HIGH);
client.println("HTTP/1.1 200 OK");
client.println("Content-Type: text/html");
client.println("Connection: close"); // connection closed completion of response
client.println("Refresh: 10"); // refresh the page automatically every 5 sec
client.println();
client.println("<!DOCTYPE HTML>");
client.println("<html><body>");
digitalWrite(TX_Pin,HIGH); // Turn the TX LED on
client.print("<span style=\"font-size: 26px\";><br>  Temperature is ");
client.print(ds.getTemperature_C());
client.println(" °C<br>");
client.print("%<br>  Humidity is ");
client.print(bme.getHumidity());
client.println(" %<br>");
client.print("%<br>  Pressure is ");
client.print(bme.getPressure_MB());
client.println(" mb%<br>");
client.print("%<br>  Wind Speed is ");
client.print(windSpeed);
client.println(" mph<br>");
getWindDirection();
client.print("%<br>  Direction is ");
client.print(calDirection);
client.println(" °<br>");
client.print("%<br>  Rainfall is ");
client.print(totalRainfall);
client.println(" mm<br>");
client.print("%<br>  Minimum Temp ");
client.print(minTemp);
client.println(" °C<br>");
client.print("%<br>  Maximum Temp ");
client.print(maxTemp);
client.println(" °C</span>");
client.println("</body></html>");
digitalWrite(TX_Pin,LOW); // Turn the TX LED off
break;
}
if (c == '\n') {
// you're starting a new line
currentLineIsBlank = true;
} else if (c != '\r') {
// you've gotten a character on the current line
currentLineIsBlank = false;
}
}
}
}

// give the web browser time to receive the data
delay(1);
// close the connection:
client.stop();
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
}
}

// Interrupt handler routine that is triggered when the rg-11 detects rain
void isr_rg() {

if((millis() - contactTime) > 15 ) { // debounce of sensor signal
tipCount++;
totalRainfall = tipCount * Bucket_Size;
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
