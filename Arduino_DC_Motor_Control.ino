#include <PacketSerial.h>
#include <Servo.h>

PacketSerial myPacketSerial;
Servo j_arm;

// Define the pin numbers
#define motorPin1 6
#define motorPin2 7
#define motor2Pin1 3
#define motor2Pin2 4
#define pwmMotor1 5
#define pwmMotor2 9
#define J_ARM_PIN 10

#define sRate 115200

#define robot_num 1

#if robot_num == 1
  #define j_arm_min 40 
  #define j_arm_max 120
#elif robot_num == 2
  #define j_arm_min 20 
  #define j_arm_max 120
#else
  #error "invalid robot identifier"
#endif

/*
 * 
 * Remapping the dutycycle and stuff: we're taking some input parameter/number
 * this parameter could be from -127 to 128 or somethin: 8bit 10101010
 * 0 means stop
 * negative means make the other lead high - we could take the abs val which is nice
 * I think parameter also dictates which motor for which duty cycle
 * 
 * if parameter = negative
 *   
 * 
 * duty cycle could be from 0-255 but we may not even want to consider 0-75 because the motors don't even engage for those values
 * 
 * 
 */
void moveJArm(int input) {
  if (input < 0) input = 0; 
  if (input > 100) input = 100; 
  j_arm.write(map(input, 0, 100, j_arm_min, j_arm_max));
}

void leftMotor(int input) {
  int dtyCycl = 0;
  if (input < 0) { //move backwards
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  else { //move forwards
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // motor power here
  dtyCycl = abs(input*2);
  analogWrite(pwmMotor1, dtyCycl);
}

void rightMotor(int input) { 
  int dtyCycl = 0;
  if (input < 0) { //move backwards
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } 
  else { //move forwards
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  // motor power here
  dtyCycl = abs(input*2);
  analogWrite(pwmMotor2, dtyCycl);
}

void onPacketReceived(const uint8_t* buffer, size_t size){ //what do we want to do with packet info - simply just store it in 2 parameters
  //basically read the first and second values of the array
  if (size != 3){ //this is garbage, do nothing
    leftMotor(0);
    rightMotor(0);

    Serial.println("wrong number of bytes");
    return;
  }  
  
  leftMotor((int8_t)buffer[0]);
  rightMotor((int8_t)buffer[1]);
  moveJArm((int)buffer[2]); 

  Serial.print("Arm: "); 
  Serial.println((int)buffer[2]); 
}

void setup() {
    // Set the pin modes of the above IO pins to OUTPUT
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(pwmMotor1, OUTPUT);
    pinMode(pwmMotor2, OUTPUT);
    myPacketSerial.begin(sRate);
    myPacketSerial.setPacketHandler(&onPacketReceived);
    j_arm.attach(J_ARM_PIN); 
    moveJArm(j_arm_min);

}

void loop() {
  //first we need to read the values from serial, take these values as parameters 1 and 2 and use it to control our robot
  myPacketSerial.update(); //regularly needs to do this
  
}
