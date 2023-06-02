#include <ECE3.h>

uint16_t sensorValues[8];

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int LED_RF = 41;
const int PUSHTAB = 27;

int previousPrimarySpeed = 0;
int previousSecondarySpeed = 0;
//unsigned long previousTime = 0;

boolean didDoughnut = false;
boolean secondTurnCheck = false;
int previousError = 0;
int numVars = 4;
int numDataPoints = 1500;
int VariableStore[1500][4];
int rightSpeed = 0;
int leftSpeed = 0;
int loopCount = 0;
int getL = 0;
int getR = 0;

//---------------------------------------------------------
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
  //alternative, did not work well
//  for (int i = 0 ; i< 8; i++)
//  {
//    if (i < 2100)
//    {
//      return false;
//    }
//  }
//  return true;
  int to_ret = 0;
//  ECE3_read_IR(sensorValues);
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
  getL = getEncoderCount_left();
    getR = getEncoderCount_right();
  resetEncoderCount_left();
  resetEncoderCount_right();

//do actual doghnut
  digitalWrite(left_dir_pin, HIGH);
  int getL = 0;
  int getR = 0;
  int turnSpeed = 80;
  while(getR+getL < 700)
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
  resetEncoderCount_left();
  resetEncoderCount_right();
  //if off track try hotfix
//  getL = 0;
//  getR = 0;
//  while(getR + getL < 160)
//  {
//    analogWrite(right_pwm_pin, 50);
//    analogWrite(left_pwm_pin, 50);
//    getL = getEncoderCount_left();
//    getR = getEncoderCount_right();
//  }
}

void permaStop(int initialLeft, int initialRight)
{
  ChangeBaseSpeeds(initialLeft, 0, initialRight, 0); //slow down to 0
//  delay(10000);
//  if (!digitalRead(PUSH1))
//    {
//      for (int i = 0; i < numDataPoints; i++)
//      {
////          Serial.print("error: ");
//          Serial.print(VariableStore[i][0]);
//          Serial.print("\t");
////          Serial.print("Correction Signal: ");
//          Serial.print(VariableStore[i][1]);
//          Serial.print('\t');
////          Serial.print("leftSpeed: ");
//          Serial.print(VariableStore[i][2]);
//          Serial.print('\t');
////          Serial.print(" rightSpeed: ");
//          Serial.print(VariableStore[i][3]);
//          Serial.println();
//      }
//    }
  
  while(true)
  {//infinite loop
    
  }
}

int getFusionNumber()
{
  //read in sensor values
   
  //from spreadsheet
  int minima[8] = {673, 580, 597, 597, 620, 621, 597, 713};
//  673  580 597 597 620 621 597 713
  int maxima[8] = {1827, 1921, 1903, 1903, 1880, 1879, 1903, 1787};
//  1827  1921  1903  1903  1880  1879  1903  1787
  
  int error = 0;
  int error_weight[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
  int divisor = 4;
//  int error_weight[8] =  {-15, -14, -12, -8, 8, 12, 14, 15};
//  int error_weight[8] =  {-15, -14, -12, -1, 1, 12, 14, 15};
//  int divisor = 8;
  //apply to sensor values
  for (int i = 0; i < 8; i++)
  {
//    sensorValues[i] -= minima[i];
    int mi = sensorValues[i]-minima[i];
    int maxTimesThousand = 1000 * mi / maxima[i];
    
//    Serial.print("error: ");
//    Serial.print(error);
//    Serial.print("\t sensor -mi: ");
//    Serial.print(mi);
//    Serial.print("\t error_weight: ");
//    Serial.print(error_weight[i]);
//    Serial.print("\t sensor value: ");
//    Serial.print(sensorValues[i]);
//    Serial.println();

    //i been seeing ppl just do error+=sensor*weight and doing well --> may want to test that
//          error += sensorValues[i]*error_weight[i];

    error = error + error_weight[i] * maxTimesThousand;

  }
  
  return error / divisor;
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


   resetEncoderCount_left();
   resetEncoderCount_right();
//  pinMode(PUSH1, OUTPUT);
  
  Serial.begin(9600);

}

void loop()
{
//  float Kp = 0.025;
//    float Kd = 0.353;
//    int baseSpeed = 80;

//pritam's values
//  float Kp = 0.01;
//    float Kd = 0.08;
//    int baseSpeed = 60;

//    float Kp = 0.01;
//    float Kd = 0.08;
//    int baseSpeed = 80;

    float Kp = 0.04;
    float Kd = 0.16;
    int baseSpeed = 40;
    ECE3_read_IR(sensorValues);

    getL = getEncoderCount_left();
    getR = getEncoderCount_right();
    if (getL+getR > 9000 && !didDoughnut) //turnarounds
    {
      ChangeBaseSpeeds(leftSpeed, 20, rightSpeed, 20); 
      baseSpeed = 20;
    }
    else if (getL+getR > 8000 && !didDoughnut) //gap
    {
      baseSpeed = 20;
    }
//    else if (getL+getR > 3800 && getL+getR < 8000) //straightaways
//    {
////      ChangeBaseSpeeds(leftSpeed, 90, rightSpeed, 90); 
//      //Kp = 0.03;
////      Kd = 0.25;
//      baseSpeed = 60;
//    }
//    
//      for (int i = 0; i < 8; i++)
//      {
//        Serial.print(sensorValues[i]);
//        Serial.print("\t");
//      }
//      Serial.println();

//    check if at end of 1st pass
    if(turnaround() && !didDoughnut)// && secondTurnCheck
    {
      doDoughnut(leftSpeed, rightSpeed);
      didDoughnut = !didDoughnut;
      secondTurnCheck = !secondTurnCheck;
    }
    //che7688ck if at end of full track --> did two passes
    else if (turnaround() && secondTurnCheck && didDoughnut)
    {
      permaStop(leftSpeed, rightSpeed);
    }
    secondTurnCheck = turnaround();
    
    int error = getFusionNumber();
    int diffError = error - previousError; 
    
    float correctionSignal = Kp*error - Kd*diffError;
    rightSpeed = baseSpeed + correctionSignal;
    leftSpeed = baseSpeed - correctionSignal;
    
  //new Ideas for keeping the speed values within range
    if(rightSpeed < 0)
      rightSpeed = 15;
    else if (rightSpeed > 255)
      rightSpeed = 250;

    if(leftSpeed < 0)
      leftSpeed = 15;
    else if (leftSpeed > 255)
      leftSpeed = 250;

    analogWrite(right_pwm_pin, rightSpeed);
    analogWrite(left_pwm_pin, leftSpeed);
    previousError = error;
//        if(loopCount < numDataPoints)
//    {
//      VariableStore[loopCount][0] = error;
//      VariableStore[loopCount][1] = correctionSignal;
//      VariableStore[loopCount][2] = rightSpeed;
//      VariableStore[loopCount][0] = leftSpeed;
//    }
//    loopCount++;
         
//        Serial.print(error);
//        Serial.print("\t");
//        Serial.print(diffError);
//        Serial.print("\t");
//        Serial.print(correctionSignal);
//        Serial.print("\t");
//        Serial.print(leftSpeed);
//        Serial.print("\t");
//        Serial.println(rightSpeed);
//         Serial.println();
//      delay(100);

}
