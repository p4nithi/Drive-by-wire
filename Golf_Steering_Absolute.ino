#define CLK 10 // Blu Digital Pin 10
#define DO 11 // Grn Digital Pin 11
#define CSn 12 // Ylw Digital Pin 12
#define pwm1 9  // PWM motor
#define dir 8   // Direction 
#define CH1 7 // X axis CH1 RC remote
#define CH7 6 // Safety switch
#include <PID_v1.h>
#include <Smoothed.h>
Smoothed <int> steering;

int rp = 0;
String readString; //This while store the user input data
int User_Input = 0; // This while convert input string into integer
int MAX_Steering = 1600;
int RC_min = 990;
int RC_max =1990; 

double kp = 1.1 , ki = 0.03 , kd = 0.01;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

int absolute_position;
int Degree;
int Deg = 0;
int RC1_X;
int  SafetySW;
void setup()
{ pinMode(pwm1, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(CSn, OUTPUT);// Chip select
  pinMode(CLK, OUTPUT);// Serial clock
  pinMode(DO, INPUT_PULLUP);// Serial data IN/OUT
  pinMode(CH7, INPUT_PULLUP); 
  
  pinMode(CH1, INPUT);// Serial clock
  steering.begin(SMOOTHED_AVERAGE, 5);

  digitalWrite(CSn, HIGH);
  digitalWrite(CLK, HIGH);
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(0.2);  // refresh rate of PID controller
  myPID.SetOutputLimits(-100, 100); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  Serial.begin(115200);
  Serial.print("Golfcart steering will start in 10 secound "); 
  Serial.print("*********************************************");
  delay(10000);
}




void loop() {
  SafetySW = pulseIn(CH7, HIGH, 25000);
  int Data = ReadSSI();
  Degree = map(Data, 0, 4095, 0, 359);
  update_absolute_position(Degree);
  Deg = (rp * 360) + absolute_position;
  
  Serial.print("steering = "); Serial.print(setpoint);
  Serial.print(", Deg = "); Serial.println(Deg);
  finish ();
   
  
  RC1_X = pulseIn(CH1, HIGH, 25000);
  steering.add(RC1_X);
  double smooth_steering = steering.get();

  

  while (Serial.available()) { //Check if the serial data is available.
    delay(3);                  // a small delay
    char c = Serial.read();  // storing input data
    readString += c;         // accumulate each of the characters in readString
  }

  if (readString.length() > 0) { //Verify that the variable contains information
    Serial.print("Input: ");
    Serial.print(readString.toInt());  //printing the input data in integer form
    Serial.print("\t");
    User_Input = readString.toInt();   // here input data is store in integer form
  }


  if (smooth_steering < 1490){ setpoint = map(smooth_steering, 1490, RC_min, 0, -MAX_Steering); }
  else if (smooth_steering > 1510){ setpoint = map(smooth_steering, 1502, RC_max, 0, +MAX_Steering); }
  else {setpoint=0;}
  
  input = Deg;
  myPID.Compute();                 // calculate new output
  
  
//  while(SafetySW > 2000)
//        { 
            pwmOut(output);
            SafetySW = pulseIn(CH7, HIGH, 25000);
//        }



  Serial.print("smooth = "); Serial.print(smooth_steering);
  Serial.print("steering = "); Serial.print(setpoint);
  Serial.print(", Deg = "); Serial.println(Deg);
 // Serial.print(", out = "); Serial.println(output);
  //Serial.print(", Delta = "); Serial.println(delta);
 // Serial.print(", smooth_steering = "); Serial.println(smooth_steering);


  

}

void pwmOut(int out) {
  if (out > 0) {                         // if REV > encoderValue motor move in forward direction.
    analogWrite(pwm1, out);         // Enabling motor enable pin to reach the desire angle
    CCW();                           // calling motor to move forward
  }
  else if (out  < 0) {
    analogWrite(pwm1, abs(out));          // if REV < encoderValue motor move in forward direction.
    CW();                            // calling motor to move reverse
  }
  else {
    finish () ;
  }

  readString = ""; // Cleaning User input, ready for new Input
}



void CCW () {
  digitalWrite(dir, LOW);

}
void CW () {
  digitalWrite(dir, HIGH);

}
void finish () {
  analogWrite(pwm1, 0);          // if REV < encoderValue motor move in forward direction.


}


int ReadSSI(void)
{ int i, dReading;
  char Resolution = 12;
  unsigned int bitStart = 0x0800;
  dReading = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(5);
  digitalWrite(CLK, LOW);
  for (i = (Resolution - 1); i >= 0; i--)
  { digitalWrite(CLK, HIGH);
    delayMicroseconds(5);
    if (digitalRead(DO)) dReading |= bitStart;
    digitalWrite(CLK, LOW);
    delayMicroseconds(5);
    bitStart = bitStart >> 1;
    if (i == 0)
    { digitalWrite(CLK, HIGH);
      if (digitalRead(DO)) dReading |= bitStart;
    }
  }
  digitalWrite(CSn, HIGH);
  return dReading;
}


void update_absolute_position(int encoder_position)
{
  int old_position = absolute_position;  // extract LSB
  int delta = encoder_position - old_position;
 if (delta > 320) {
    rp--;
  }
  else if (delta < -320) {
    rp++;
  }
 //Serial.print(delta); Serial.print("  ,");
//  Serial.print(rp); Serial.print("  ,");

//  if (delta > 0) {
//    Serial.print("CW,  ");
//  }
//  else if (delta < 0) {
//    Serial.print("CCW,  ");
//
//  }
//  else {
//    Serial.print("Stop,  ");
//  }


  absolute_position += delta;

 

}
