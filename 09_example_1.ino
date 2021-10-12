// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.5 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.
#define M 3
#define N_MAX 30
int n_list[M]={3,10,30};

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw,first,memo[M]; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float dist_list[N_MAX];
float sorted_dist[M][N_MAX];
int m,n,n_max,i;
float median;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
// initialize serial port
  Serial.begin(57600);

  m=M;
  n_max=N_MAX;
// initialize last sampling time
  last_sampling_time = 0;
  first = USS_measure(PIN_TRIG,PIN_ECHO);
  for (i=0; i<n-1; i++) {
    dist_list[i]= first;
  }
  for(int j=0;i<m;j++){
    n=n_list[j];
    for (i=0; i<n-1; i++) {
      sorted_dist[j][i]= first;
    }
  }
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_list[n_max-1]=dist_raw;

  // output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");

  
  for(int j=0;j<m;j++){
    n=n_list[j];
  for(i=0;i<n-1;i++){
    if(memo[j]==sorted_dist[j][i]){
      break;
    }
  }
  for(;i<n-1;i++){
    sorted_dist[j][i]=sorted_dist[j][i+1];
  }

  for(i=n-1;i>0;i--){
    if(sorted_dist[j][i-1]<=dist_raw){
      break;
    }
    sorted_dist[j][i]=sorted_dist[j][i-1];
  }
  sorted_dist[j][i]=dist_raw;
  
  if (n%2==0) {
    median = (sorted_dist[j][n/2-1] + sorted_dist[j][n/2])/2;
  }
  else {
    median = sorted_dist[j][n/2];
  }
  memo[j]=dist_list[n_max-n];
  



  Serial.print("median");
  Serial.print(n);
  Serial.print(":");
  Serial.print(map(median,0,400,100+50*j,500+50*j));
  Serial.print(",");
  }
  
  for (i=0; i<n-1; i++) {
    dist_list[i]=dist_list[i+1];
  }
  Serial.println("Max:600");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0; // return 0 when out of range.
  return reading;
}
