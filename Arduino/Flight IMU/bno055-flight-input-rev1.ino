#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>


/* This is the main "flight" software for the balance bot (v4). At the core is a BNO055 IMU (via adafruit). Uses its built in kalman filters
   we are feeding it through a PID loop to try and always keep the pitch axis level. We do this by moving the wheels "under" the IMU at all times.
   If you tilt forward 10 degrees it should accelerate in that direction so it can try and get the feet back under it.
   It is also the i2c master. However the bno055 library from adafruit starts i2c and so its 'missing' from this code. 
*/



// Setup the BNO055 sensor. Set to 100hz with 10ms delay.
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Setup PID Variables
double Setpoint, Input, Output;
double movSetpoint, movInput, movOutput;
double SetpointBase = 0.00;

double consKp = 1.15, consKi = 0 , consKd = 0; // In Conjuction with the encoder and the revised output scheme a low value for P is critical.
double movKp = .15, movKi = 0, movKd = 0; // PID values for the movement engine

/* 
 *  Idea to stabilize: Implement a movement system. The basic idea is that we seperate the 'balance PID' from the overall movement. 
 *  When something is set to "movement 0", if you will, then the system will try and keep the encoder movement count near 0. 
 *  Im not 100% sure if the 'correcting' mechanism should be a + - modifier to the setpoint? 
 *  Initial tests kind of work. It seems to sway forward and back instead of just running away in one direction. Perhaps some PID tuning?
 */

// Setup pin variables


byte iByte;
byte sByte;
int Count = millis();
int PwmDeadband = .5;
int MovCounter = 0; //Variable to count number of IMU cycles. Used to determine when to run the movement PID.


union encoder1_tag {  // To rebuild the I2C floats
   byte b[4];
   int iv; // float Value
} encoder1;

union encoder2_tag {  // To rebuild the I2C floats
   byte b[4];
   int iv; // float Value
} encoder2;


union i2c_double_tag{  // To rebuild the I2C floats
   byte b[4];
   double dv; // Double value
} i2cDouble;

PID MyIMUFlightPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, P_ON_E, REVERSE);
PID MyMovementPID(&movInput, &movOutput, &movSetpoint, movKp, movKi, movKd, P_ON_E, DIRECT);

void IMUCalGet(void)
{

  // Outputs the sensors calibration ratings for each system. Greater than 0 is acceptable. 3 is highest.
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("Calibration Data: (System/Gyro/Accel/Mag)  "); Serial.print(system, DEC); Serial.print(" / "); Serial.print(gyro, DEC); Serial.print(" / "); Serial.print(accel, DEC); Serial.print(" / "); Serial.println(mag, DEC);

}


void serialUpdate() {


  iByte = Serial.read(); // First byte to check


  if (iByte == 65) { // If this is A thats our header.

    sByte = Serial.read(); // Data from packet. The type part.
    
    Serial.println(sByte);
    if (sByte == 66) {
      
      Setpoint += .5;
      Serial.print("Up: ");
      Serial.println(Setpoint);


    }
     else if(sByte == 80) {
           
           Serial.print("P");
           
           Serial.println(consKp);
           
            MyIMUFlightPID.SetTunings(consKp, consKi, consKd,P_ON_E);
     }
      else if(sByte == 73) {

          consKi = Serial.read();
          //    Serial.println(consKi);
           MyIMUFlightPID.SetTunings(consKp, consKi, consKd,P_ON_E);
     }
      else if(sByte == 68) {
            consKd = Serial.read();
            MyIMUFlightPID.SetTunings(consKp, consKi, consKd,P_ON_E);
     }
    else if (sByte == 88) {
        Setpoint -= .5;
        Serial.println("Down");
       Serial.println(Setpoint);
    }
  }
  else if (iByte == 10) {
  }
  else {
    //  Serial.println("Serial sync error.");
  }
}


void setup() {
  
  
  Setpoint = 0;
  SetpointBase = Setpoint;

  // Set PID data and start it


  MyIMUFlightPID.SetSampleTime(10);  // Set to 10ms delay so that I can run it right after a sensor grab (which occur every 10ms).1
  MyIMUFlightPID.SetTunings(consKp, consKi, consKd, P_ON_E);
  MyIMUFlightPID.SetOutputLimits(-56, 56); // Output is set to max mm/10ms range. 

  MyMovementPID.SetSampleTime(200); // 200ms check delay. 5 times a second. Too fast and it will rapidly overcorrect for just balance corrections.
  MyMovementPID.SetTunings(movKp, movKi, movKd, P_ON_E);
  MyMovementPID.SetOutputLimits(-5,5); // Really small outputs. Otherwise we could have this just get smashed into the floor. 

  MyMovementPID.SetMode(AUTOMATIC);   
  MyIMUFlightPID.SetMode(AUTOMATIC);

  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("Error. Failure to find BNO.");
    while (1); // Borrowed from adafruit
  }
  delay(1000);

  bno.setExtCrystalUse(true); // I believe this is just to let the bno sensor know its on a board with external crystal

}

// put your setup code here, to run once:


void encoder_request() {
    Wire.requestFrom(8, 4);
  
  while (Wire.available()){
    // char EncoderInput = Wire.read();
   
    encoder1.b[0] = Wire.read();
    encoder1.b[1] = Wire.read();
    encoder1.b[2] = Wire.read();
    encoder1.b[3] = Wire.read();    
    //Serial.print("Encoder Input(I2C): "); Serial.println(encoder1.iv); // 
  }

   Wire.requestFrom(9, 4);
  
  while (Wire.available()){
    // char EncoderInput = Wire.read();
   
    encoder2.b[0] = Wire.read();
    encoder2.b[1] = Wire.read();
    encoder2.b[2] = Wire.read();
    encoder2.b[3] = Wire.read();    
    //Serial.print("Encoder Input(I2C) REAR: "); Serial.println(encoder2.iv); // 
  }

}



void loop() {

  
  if (Serial.available() > 1) {
    Serial.println("WOrks");
    serialUpdate();

  }

  // Call sensors_event_t event; then bno.getEvent(&event); to populate it

  if  (millis() - Count >= 10) {
    sensors_event_t event;
    bno.getEvent(&event);

    
    Input = event.orientation.y;
    //Serial.println(Input);
    
    Count = millis();
    MyIMUFlightPID.Compute(); 
    //Serial.println(Output);
    Wire.beginTransmission(8);
    i2cDouble.dv = Output;
    Wire.write(i2cDouble.b[0]);
    Wire.write(i2cDouble.b[1]);
    Wire.write(i2cDouble.b[2]);
    Wire.write(i2cDouble.b[3]);
    Wire.endTransmission();
    Wire.beginTransmission(9);
    Wire.write(i2cDouble.b[0]);
    Wire.write(i2cDouble.b[1]);
    Wire.write(i2cDouble.b[2]);
    Wire.write(i2cDouble.b[3]);
    Wire.endTransmission();
   //Serial.println(i2cDouble.dv);
   MovCounter = MovCounter + 1;
   Serial.print(Input); Serial.print(" , "); Serial.println(Output);
    }
   /*
    * 
    
     if (MovCounter >= 20){
      encoder_request(); // Get Encoder value updates from the motor arduinos
      movInput = ((double) encoder1.iv)/200; // Take the encoder movements that have occured and divide by 100. 100 steps = 13.9mm wheel traveled. 
      MyMovementPID.Compute();
      
      
      Setpoint = SetpointBase + movOutput;
      Serial.print(movOutput); Serial.print(" , "); Serial.print(movInput); Serial.print(" , "); Serial.println(Setpoint);
      MovCounter = 0;
    }
    */
   
}
