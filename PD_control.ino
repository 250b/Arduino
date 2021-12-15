#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9         // LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10    // 서보모터를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0     // 적외선 센서를 아두이노 아날로그 A0핀에 연결

// Framework setting
#define _DIST_TARGET 220     
#define _DIST_MIN 100   //거리의 최솟값이 100mm
#define _DIST_MAX 410                  //거리의 최대값 410mm
#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.1     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
#define _DIST_ALPHA 0.35
// Servo range
#define _DUTY_MIN 1440       
#define _DUTY_NEU 1500       
#define _DUTY_MAX 1560             // 서보의 최대각도의 microseconds의 값

// Servo speed control
#define _SERVO_ANGLE 30   //  최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 2000   //서보 속도를 30으로

// Event periods
#define _INTERVAL_SERVO 20  //  서보 업데이트 주기
#define _INTERVAL_SERIAL 100  //  시리얼 플로터 갱신 속도

// PID parameters
#define _KP 0.7  //  P 이득 비율
#define _KD 70

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

float dist_target; 
float dist_f;
float ema_dist=0;            
float filtered_dist;       
float samples_num = 3;     
int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum, dist_ema, alpha, dist_cali_f;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;

//(unsigned long : 부호없는 정수형)
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; //  서보속도 제어를 위한 변수 선언
int duty_target, duty_curr; //  목표duty, 현재duty

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); 
  myservo.attach(PIN_SERVO); 

// initialize global variables
duty_target = _DUTY_NEU;
duty_curr = _DUTY_NEU; // [3055] duty_target, duty_curr 초기화
last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = millis();
dist_f, dist_ema = _DIST_MIN; 
pterm = iterm = dterm = 0;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); //[2635]

// initialize serial port
  Serial.begin(57600); //[2635]

  a = 80;
  b = 280;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;
  
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO;   

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
// ================
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ 

  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
    sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);
  dist_cali_f=alpha*dist_cali + (1-alpha)*dist_ema;
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

void loop() {
    /////////////////////
    // Event generator //
    /////////////////////
    unsigned long time_curr = millis(); 
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    ////////////////////
    // Event handlers //
    ////////////////////

    if(event_dist) {
        event_dist = false; 
        // get a distance reading from the distance sensor
        dist_f = filtered_ir_distance(); 
        
        // PID control logic
        error_curr = _DIST_TARGET-dist_f; 
        pterm = error_curr;
        dterm = _KD*(error_curr - error_prev);
        control = _KP * pterm+dterm;

        // duty_target = f(duty_neutral, control)
        //duty_target = duty_neutral + control 
        duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN) 
        {
            duty_target = _DUTY_MIN;
        }
        if(duty_target > _DUTY_MAX)
        {
            duty_target = _DUTY_MAX;
        }

        error_prev=error_curr;
  }
  
 if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
   duty_curr += duty_chg_per_interval;
   if(duty_curr > duty_target) duty_curr = duty_target;
     }
    else {
   duty_curr -= duty_chg_per_interval;
   if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_f);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
   
  }
}
