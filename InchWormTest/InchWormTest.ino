
#include <Servo.h>
// 177 176 180

Servo motorA;
Servo motorB;
Servo motorC;
int pinA = 10;
int pinB = 11;
int pinC = 9;

int val = 0;
char buf[9];
int len = 9;
char temp[3];
int tempIdx;
int jointAngles[3];
int jointAngleIdx;
int minPWM = 500;
int maxPWM = 2500;

void setup() {
  // put your setup code here, to run once:
  motorA.attach(pinA, minPWM, maxPWM);
  motorB.attach(pinB, minPWM, maxPWM);
  motorC.attach(pinC, minPWM, maxPWM);
  Serial.begin(9600);
  motorA.write(90);
  motorB.write(180);
  motorC.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
    Serial.println("Message received");
    Serial.readBytesUntil('\n', buf, len);
    tempIdx = 0;
    jointAngleIdx = 0;

    if(buf[0] == '!'){
      delay(3000);
      for(int j = 0; j < 8; j++){ 
        motorA.write(155);
        motorB.write(56);
        motorC.write(150);
        delay(2000);
        motorA.write(139);
        motorB.write(89);
        motorC.write(132);
        delay(2000);
//        motorA.write(150);
//        motorB.write(56);
//        motorC.write(156);
//        delay(3000);
      }
//      motorA.write(180);
//      motorB.write(180);
//      motorC.write(180);
    }
    else{
      for(int i = 0; i < len; i++){
        temp[tempIdx] = buf[i];
        
        if(tempIdx < 2){
          tempIdx++;
        }
        else{
          jointAngles[jointAngleIdx] = atoi(temp);
          jointAngleIdx++;
          tempIdx = 0;
        }
      }
  
      Serial.println("Degree :");
      Serial.println(jointAngles[0]);
      Serial.println(jointAngles[1]);
      Serial.println(jointAngles[2]);
      motorA.write(jointAngles[0]);
      motorB.write(jointAngles[1]);
      motorC.write(jointAngles[2]);
    }
  }
  Serial.println("Nothing yet");
  delay(300);
}
