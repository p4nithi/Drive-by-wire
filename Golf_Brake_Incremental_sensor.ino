#define encoderPin1 3  //encoder White
#define encoderPin2 2  //encoder Green 

#define pwm1 9  // PWM motor
#define dir 8   // Direction 
#define CH2 7 // Y axis CH2 RC remote
#define CH7 6 // Safety switch

#define PWM_CH3 10

#define ss_l 12
#define ss_r 4


#include <PID_v1.h>
#include <Smoothed.h>
Smoothed <int> throttle;

int rp = 0;
String readString; //This while store the user input data
int User_Input = 0; // This while convert input string into integer
int MAX_throttle = 45;
double kp = 5  , ki = 2.5 , kd = 0.8;            // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

int absolute_position;
int Degree;
int Deg = 0;
int RC2_Y;
int  SafetySW;

volatile long encoderValue = 0;
volatile long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
volatile int lastEncoded = 0;
volatile int rotation = 0;
volatile long OT = 0;
volatile long angle = 0;
volatile long RPM = 0;
int t = 0, i = 0;


void setup()
{ pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(PWM_CH3, INPUT_PULLUP);

  pinMode(pwm1, OUTPUT);
  pinMode(dir, OUTPUT);
  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);


  pinMode(ss_l, INPUT_PULLUP);
  pinMode(ss_r, INPUT_PULLUP);
  pinMode(CH7, INPUT_PULLUP);
  pinMode(CH2, INPUT);// Serial clock
  throttle.begin(SMOOTHED_AVERAGE, 5);


  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(0.05);  // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  Serial.begin(115200);
}




void loop() {
  finish ();


  RC2_Y = pulseIn(CH2, HIGH, 40000);
  throttle.add(RC2_Y);
  double smooth_throttle = throttle.get();


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


  // if (smooth_throttle < 1492){ setpoint = map(smooth_throttle, 1492, 1082, 0, +MAX_throttle); }
  if (smooth_throttle > 1600) {
    setpoint = map(smooth_throttle, 1502, 1988, 0, MAX_throttle);
  }
  else {
    setpoint = 0;
  }


  //  angle = encoderValue * 0.277;
  //  input = angle;
  //  myPID.Compute();                 // calculate new output
  //  pwmOut(output);

  int brake_pwm = map(smooth_throttle, 1492, 1080, 0, 255);
CCW();
  analogWrite(pwm1, brake_pwm);
  if (digitalRead(ss_l) == 0 || digitalRead(ss_r) == 0) {
  CCW();
  analogWrite(pwm1, 255);
  Serial.print("Brake !!!");
  delay(2000);
  }
  else{  analogWrite(pwm1,0);}
 
  Serial.print(", smooth_throttle = "); Serial.print(brake_pwm);
  Serial.print("L = "); Serial.print(digitalRead(ss_l)); Serial.print("\tR = "); Serial.print(digitalRead(ss_r));
  Serial.print(", Brake = "); Serial.print(setpoint);
  Serial.print(", Deg = "); Serial.println(angle);
  //Serial.print(", out = "); Serial.print(output);
  //Serial.print(", Delta = "); Serial.println(delta);

}

void pwmOut(int out) {
  if (setpoint > angle ) {                         // if REV > encoderValue motor move in forward direction.
    analogWrite(pwm1, out);         // Enabling motor enable pin to reach the desire angle
    CCW();                           // calling motor to move forward
    // Serial.println(", CCW ");
  }
  else if (setpoint < angle && angle > 0) {
    analogWrite(pwm1, abs(out));          // if REV < encoderValue motor move in forward direction.
    CW();                            // calling motor to move reverse
    //Serial.println(", CW ");
  }

  else if (setpoint == angle || angle < 0) {
    // Serial.println(", Finish ");
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





void updateEncoder() {
  int MSB = digitalRead(encoderPin1);
  int LSB = digitalRead(encoderPin2);
  int encoded = (MSB << 1) | LSB;
  int sum  = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded;
}
