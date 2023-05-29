#include <ECE3.h>

uint16_t sensorValues[8];

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int LED_RF = 41;
int previousError = 0;
int previousPrimarySpeed = 0;
int previousSecondarySpeed = 0;
unsigned long previousTime = 0;


void ChangeBaseSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd)
{
  /*
    This function changes the car speed gradually (in about 30 ms) from
    initial
    speed to final speed. This non-instantaneous speed change reduces the
    load
    on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd - initialLeftSpd;
  int diffRight = finalRightSpd - initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft) / stepIncrement;
  int numStepsRight = abs(diffRight) / stepIncrement;
  int numSteps = max(numStepsLeft, numStepsRight);
  int pwmLeftVal = initialLeftSpd; // iniialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft) / numSteps; // left in(de)crement
  int deltaRight = (diffRight) / numSteps; // right in(de)crement
  for (int k = 0; k < numSteps; k++)
  {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin, pwmLeftVal);
    analogWrite(right_pwm_pin, pwmRightVal);
    delay(30);
  } // end for int k
  analogWrite(left_pwm_pin, finalLeftSpd);
  analogWrite(right_pwm_pin, finalRightSpd);
} // end void ChangeWheelSpeeds

boolean turnaround()
{
//  for (int i = 0 ; i< 8; i++)
//  {
//    if (i < 2100)
//    {
//      return false;
//    }
//  }
//  return true;
  int to_ret = 0;
  ECE3_read_IR(sensorValues);
  for (int i = 0; i< 8; i++)
  {
    to_ret = to_ret + sensorValues[i];
//    Serial.print(sensorValues[i]);
//    Serial.print("\t");
  }
//  Serial.println();
//  Serial.print("to_ret: ");
//  Serial.print(to_ret);
//  Serial.println();
  return to_ret > 16000; //2500*8 = 20000 but we do a little under
  
}

void doDoughnut(int initialLeft, int initialRight)
{
  ChangeBaseSpeeds(initialLeft, 0, initialRight, 0); //slow down to 0

  resetEncoderCount_left();
  resetEncoderCount_right();

  digitalWrite(left_dir_pin, HIGH);
  int getL = 0;
  int getR = 0;
  int turnSpeed = 80;
  while(getR+getL < 720)
  {
    analogWrite(right_pwm_pin, turnSpeed);
    analogWrite(left_pwm_pin, turnSpeed);
    getL = getEncoderCount_left();
    getR = getEncoderCount_right();
  }

  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
  
  digitalWrite(left_dir_pin, LOW);
  ChangeBaseSpeeds(0, initialLeft, 0, initialRight); //speed up back to initial
}

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  uint16_t sensorValues[8];

  //  pinMode(LED_RF, OUTPUT);
  Serial.begin(9600);
//  previousTime = millis();
//   doDoughnut(0, 0);
}

void loop() {
    int baseSpeed = 20;
    
//    for (int i = 0 ; i< 8; i++)
//    {
//     Serial.print(sensorValues[i]);
//     Serial.print("\t");
//    }
//    Serial.println();
    analogWrite(right_pwm_pin, baseSpeed);
    analogWrite(left_pwm_pin, baseSpeed);
    if (turnaround())
    {
      doDoughnut(baseSpeed, baseSpeed);
    }
//    Serial.println(turnaround());
//    delay(100);
    
   
}
