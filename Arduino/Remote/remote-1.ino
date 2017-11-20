#include <SoftwareSerial.h>
// SoftwareSerial Serial1(0, 1); 
// creates a "virtual" serial port/UART
// connect Serial1 module TX to D10
// connect Serial1 module RX to D11
// connect Serial1 Vcc to 5V, GND to GND



const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A1;  // Analog input pin that the potentiometer is attached to
const int analogInPin3 = A2;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
const int yAxisInPin = A8;
const int xAxisInPin = A9;

int sensorValue = 0;        // value read from the pot
int sensorValue2 = 0;
int sensorValue3 = 0;
int xSensorValue = 0;
int ySensorValue = 0;


void setup()  
{
  Serial1.begin(9600);
  // set digital pin to control as an output
  pinMode(13, OUTPUT);
  // set the data rate for the SoftwareSerial port
  Serial1.begin(9600);
  // Send test message to other device
  Serial1.println("Hello from Arduino");
}
char a; // stores incoming character from other device
void loop() 
{
  sensorValue = analogRead(analogInPin);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3);
  xSensorValue = analogRead(xAxisInPin);
  ySensorValue = analogRead(yAxisInPin);

  Serial.println(sensorValue);
  Serial1.print(sensorValue); Serial1.print(" "); Serial1.print(sensorValue2); Serial1.print(" "); Serial1.print(sensorValue3); Serial1.print(" "); Serial1.print(xSensorValue); Serial1.print(" "); Serial1.println(ySensorValue);
  
}
