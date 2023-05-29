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
}

int getFusionNumber()
{
  //read in sensor values
  
  ECE3_read_IR(sensorValues);
//  for (int i = 0; i < 8; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print("\t");
//  }
//  Serial.println();
  //from spreadsheet
  int minima[8] = {673, 580, 597, 597, 620, 621, 597, 713};
//  673  580 597 597 620 621 597 713
  int maxima[8] = {1827, 1921, 1903, 1903, 1880, 1879, 1903, 1787};
//  1827  1921  1903  1903  1880  1879  1903  1787
  
  int error = 0;
  int error_weight[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
  int divisor = 4;
//  int error_weight[8] =  {-15, -14, -12, -8, 8, 12, 14, 15};
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
    //      error += sensorValues[i]*error_weight[i];

    error = error + error_weight[i] * maxTimesThousand;

  }
  
  return error / divisor;
}

void loop()
{
    float Kp = 0.04;
    float Kd = 0.16;
  //  float Kd = 0;
    int baseSpeed = 40;
    int error = getFusionNumber();
    int diffError = error - previousError; 
    
    float correctionSignal = Kp*error - Kd*diffError;
    int rightSpeed = baseSpeed + correctionSignal;
    int leftSpeed = baseSpeed - correctionSignal;

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
