#include <Servo.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;
Stepper stepper5(stepsPerRevolution, 4, 8, 7, 11);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo6;

int val1;
int val2;
int val3;
int val4;
int val6;


//stepper constants
int val5; //stepper pot
const int minPotValue = 0;    
const int maxPotValue = 1023; 
const int minAngle = 0;
const int maxAngle = 360; 
const int minPosition = 0;
const int maxPosition = stepsPerRevolution; 
int currentPosition = 0;

//cinematica inversa
#define PI 3.1415926535897932384626433832795

float Xc = 20;
float Yc = 30;
float Zc = 30;

float L1 = 25;
float L2 = 20;
float L3 = 10;
float L4 = 15;

float theta1;
float theta2;
float theta3;
float theta4;

float r; 
float s;
float A;
float B;







void moveStepperToAngle(int angle) {
  int targetPosition = map(angle, minAngle, maxAngle, minPosition, maxPosition);
  int stepsToMove = targetPosition - currentPosition;
  stepper5.step(stepsToMove);
  currentPosition = targetPosition;
}

void setup() {

  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo6.attach(10);

  stepper5.setSpeed(10); 

  theta1 = atan2(Yc,Xc);
  r = sqrt(Xc*Xc+Yc*Yc);
  s = Zc - L1;
  A = sqrt(s*s+sq(r-L4));

  theta3 = acos((A*A-L2*L2-L3*L3)/(2*L2*L3));
  theta3 = -theta3;

  B = L3*sin(theta3);

  theta2 = atan2(s, r-L4)-asin(B/A);

  theta4 = -(theta2+theta3);



  Serial.begin(9600);

}

void loop() {

  // val1 = map(val1, 0, 1023, 0, 180);   
  servo1.write(theta1*180/PI);            
  delay(15);                           

  val2 = theta2*180/PI;            
  val2 = map(val2, -95, 85, 0, 180);     
  servo2.write(val2);                 
  delay(15);  

  val3 = theta3*180/PI;            
  val3 = map(val3, 15, -165, 0, 180);     
  servo3.write(val3);       
  delay(15); 


  val4 = theta4*180/PI;         
  val4 = map(val4, -50, 105, 0, 180);   
  servo4.write(val4);            
  delay(15); 
           
  // val6 = map(val6, 0, 1023, 0, 180);     
  servo6.write(170);                
  delay(15); 


  val5 = analogRead(A4);
  int angle = map(val5, minPotValue, maxPotValue, minAngle, maxAngle);
  moveStepperToAngle(angle);
  delay(15);


}



