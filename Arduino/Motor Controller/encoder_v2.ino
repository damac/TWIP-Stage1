// Quadrature encoder 4 phase counter system.
// Attaching both encoder lines to interrupts I allow it to trigger every change that occurs.
// This in conjuction with the below phases allow you to get all the moves.
// PID Details & concept: We need to include a PID so that we can set a movement aim and let the PID try to get there.
// So now instead of the IMU sending the PWM value to try and move it will send a goal speed.
// We'll need to do some basic math to get all that working but we know its "1920" steps per rotation and we have a wheel size
// of 85mm outside diameter. Giving us 85*Pi~= 267.035 mm per rotation for wheel. Meaning each step is ~=.13908mm a step.
// Daniel Maciulla, 10/30/17

/* Testing Revealed that at 225 PWM with 16v or so we are getting roughly 5.6mm/ms (560mm/second) without any load. Halfing that I would say 2.3mm/ms is probably near 'top speed' for it
    I will add that into the PID as its upper limit and then reset the way it gets orders. Input = encoder movement. Setpoint = mm/ms desired.
    I changed it to work on a 10ms frequency so that I can get a more effective range from the PID. (56 vs 5.6 with .00 res.)
*/

#include<PID_v1.h>
#include<Wire.h>


// http://www.dynapar.com/Technology/Encoder_Basics/Quadrature_Encoder/ -- For mroe details
// in order to accurately track all 4 phases as described below:
// Phase 1: A high, B low, Previous_A low Previous_B low
// Phase 2: A high, B high, P_A high, P_B low
// Phase 3: A low, B high, P_A high, P_B high
// Phase 4: A low, B low, P_A low, P_B high
// Moving from 1 phase to another is 1/4 turn.

int encoder_A = 2;
int encoder_B = 3;
int PCog = 0;   // Encoder_A is first interrupt, _B is second, PCog is position of cog (ie number of steps)
int Change;
int A_Pin = LOW;
int B_Pin = LOW;
int Last_A_Pin = HIGH;
int Last_B_Pin = HIGH;
int SystemPhase = 0;
int currentPhase = 0;
int j;
int NewMsg;
int Sbase;
int Red = 6;
int MoveOutput;
int Black = 7;
int en1 = 10;
int PWMDeadband = 10;
int PIDFreqMS = 5;
int PIDFreq = 0;
//int PcogC;

  
union i2c_inc {  // To rebuild the I2C data
  byte b[4];

  double dv; // float Value
} i2cDouble;

union scog { // Hold SCOG and ease i2c comms
  byte b[4];
  float value; // hold the scog value (PCog * StepsTaken / Time for mm/Xms where X is the time. normally 10ms.);
  double dvalue;
} Scog;

union pcogc { 
  byte b[4];
  int value;
} PCogC;

double Setpoint, Input, Output;
double consKp = 25, consKi = 4, consKd = 0;
float wheelSize = 267.04; //85mm wheel
float StepsPerRotation = 1920; // Gearing
float StepDistance = (wheelSize / StepsPerRotation);
int  TimeBase = 0;
int TimeDelta = 0;
int MotorSpeed = 0;

int i;

PID DCMovementPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, P_ON_E, REVERSE); // Create PID


int move_motors(int value) {

  // Check which direction we should go and then set motors in that direction. Then apply PWM (en1).

  if (value >= 0) {
    digitalWrite(Red, HIGH);
    digitalWrite(Black, LOW);
    analogWrite(en1, value + PWMDeadband);
  }
  else
  {
    digitalWrite(Red, LOW);
    digitalWrite(Black, HIGH);
    value = abs(value);
    analogWrite(en1, value + PWMDeadband);

  }
  //Serial.println(value + PWMDeadband);
  return 0;
}

void setup() {
  Wire.begin(8); // FORWARD NANO IS "8". REAR NANO IS "9".
  Wire.onReceive(recieveEvent); //  Receives a transmission from a master.
  Wire.onRequest(requestEvent); // Register a function to be called when a master requests data from this slave device.

  DCMovementPID.SetSampleTime(PIDFreqMS);
  DCMovementPID.SetTunings(consKp, consKi, consKd, P_ON_E);
  DCMovementPID.SetOutputLimits((-245 + PWMDeadband), (245 - PWMDeadband)); // The limit needs to match up with the PWMDeadband so you dont request out of spec ranges.
  DCMovementPID.SetMode(AUTOMATIC);
  pinMode (encoder_A, INPUT);
  pinMode (encoder_B, INPUT);
  pinMode (Red, OUTPUT); // Enable motor drive pins.
  pinMode (Black, OUTPUT);
  pinMode (en1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_A), phase_shift, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_B), phase_shift, CHANGE);
  Serial.begin (115200);
  Sbase = 0;
  //Serial.println(StepDistance);
  PIDFreq = 10/PIDFreqMS; // PIDFrequency is runs per second. So PIDFreqMS being 5 means PidFreq is 2. 
}

void requestEvent() {


  //Serial.print("Request Event sent: "); Serial.println(Scog.value);
  for (i = 0; i < 3; i++) { // Should send all the bytes (4) to the controller.
    Wire.write(PCogC.b[0]);
    Wire.write(PCogC.b[1]);
    Wire.write(PCogC.b[2]);
    Wire.write(PCogC.b[3]);
  }
  PCogC.value = 0;
  //Wire.write("hello ");

}



void recieveEvent(int BufferSize) { // Event to be ran when we recieve messages from master. This is recieving a double from the master.

  if (BufferSize >= 4) {
    i2cDouble.b[0] = Wire.read();
    i2cDouble.b[1] = Wire.read();
    i2cDouble.b[2] = Wire.read();
    i2cDouble.b[3] = Wire.read();
  }
 // PCog = 0; // Reset encoder counter for new message. 
  NewMsg = 1;
  //Setpoint = (i2cDouble.dv / 100 ); // Should be updated here. 
  //Serial.println(i2cDouble.dv);

}


// Main Loop needs to call PID Update every loop and then let the PID library do its own timing.
// Also it will need to accept incoming I2C and everything else.

void loop() {
  if (NewMsg == 1){
    NewMsg = 0;
   //Setpoint = ((i2cDouble.dv / 10));
   
   //Serial.println(PCog);
   //PCog = 0;
  }

  TimeDelta = millis() - TimeBase;
  if (TimeDelta >= PIDFreqMS) {
    PCogC.value = PCogC.value + PCog;
    //Serial.print(Setpoint); Serial.print(" "); Serial.println(Scog.dvalue);
    Scog.value = PCog / TimeDelta; // Including StepDistance at this point just mucks up the math.
    
    TimeBase = millis();
    //Serial.print("TimeDelta: ");Serial.print(TimeDelta);Serial.print(" Pcog: "); Serial.println(PCog);
    Setpoint = i2cDouble.dv;
    
    //Serial.print("Setpoint: "); Serial.print(Setpoint); Serial.print("  Scog.dvalue: ");Serial.print(Scog.dvalue); Serial.print(" Output: ");Serial.println(Output);
    //Input = Scog.dvalue*-1;  // Keep in mind Scog value is at 2ms. Giving it a cap of like 5.6mm/10ms OR 1.12 .  ***SET TO OTHER LINE FOR 8. NINE GETS NEGATIVE***
    
    Input = Scog.dvalue; // Converted Scog to dvalue and feed to input. 
    
    DCMovementPID.Compute();
    MotorSpeed = int(Output); // let the movement speed be the plus whatever the output is. To establish a stable moving speed.
    Serial.print(PCogC.value);Serial.print("  "); Serial.println(Output);
    //MotorSpeed = constrain(MotorSpeed, -245, 245);
    PCog = 0;
  
    
    //Serial.print(ErrorRate); Serial.print(" "); Serial.println(Output);
  
  move_motors(MotorSpeed);
  }
  int STracker = millis()- Sbase;
  if(STracker >= 1000){
   // Serial.print("Sum over 1s: "); Serial.println(PcogC);
    //Serial.print("Movement(mm): "); Serial.println(PcogC * StepDistance);
   // Serial.print("Goal movement in mm/ms: ");Serial.println(Setpoint*StepDistance);
   // Serial.print("Actual movement in mm/ms: ");Serial.println((StepDistance * PcogC)/STracker);
    
    Sbase = millis();
    STracker = 0;
  }




  //Serial.print("MoveOutput: ");Serial.println(MoveOutput);
  // MoveOutput = -180;

   //Serial.println(Output);

}


void phase_shift() {
  int LastState = SystemPhase;
  Last_A_Pin = A_Pin;
  Last_B_Pin = B_Pin;
  A_Pin = LOW;
  B_Pin = LOW;
  A_Pin = digitalRead(encoder_A);  // Get current phases
  B_Pin = digitalRead(encoder_B);



  if ((A_Pin == HIGH) && (B_Pin == LOW)) {
    Change++;
    SystemPhase = 1; // If A is High, B is low and last time A was low with B also being low then its phase 1.
    if (LastState == 4) {
      PCog++;

    }
    else if (LastState == 2) {
      PCog--;
    }
  }
  else if ((A_Pin == HIGH) && (B_Pin == HIGH)) {
    Change++;
    SystemPhase = 2;
    if (LastState == 1) {
      PCog++;
    }
    else if (LastState == 3) {
      PCog--;
    }
  }
  else if ((A_Pin == LOW) && (B_Pin == HIGH)) {
    Change++;
    SystemPhase = 3;
    if (LastState == 2) {
      PCog++;
    }
    else if (LastState == 4) {
      PCog--;
    }
  }
  else if ((A_Pin == LOW) && (B_Pin == LOW)) {

    SystemPhase = 4;
    Change++;
    if (LastState == 3) {
      PCog++;
    }
    else if (LastState == 1) {
      PCog--;
    }
  }

  currentPhase = SystemPhase;

}


void debug_print() {
  //Serial.print("SCog: ");Serial.print(StepDistance*PCog.value);
  Serial.println(SystemPhase);
  Serial.print("PCog: "); Serial.print(PCog);
  Serial.print("A: "); Serial.print(A_Pin);
  Serial.print(" B: "); Serial.println(B_Pin);
}

