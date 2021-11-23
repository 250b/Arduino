// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10

#define PIN_IR A0
#define PIN_LED 9

Servo myservo;

int a, b; // unit: mm

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1600);
  //while(1){}
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 90; //70;
  b = 280; //300;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  if(dist_cali > 300)
  {
      myservo.writeMicroseconds(1350);
  }
  else
  {
    myservo.writeMicroseconds(1600);
  }
  delay(20);
}
