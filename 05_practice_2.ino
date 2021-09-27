#define PIN7 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN7, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  toggle = 0;
  digitalWrite(PIN7, toggle);
}

void loop() {
  digitalWrite(PIN7, LOW);
  delay(1000);
  
  for (int count=0; count < 5; count++){
    digitalWrite(PIN7, HIGH);
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);
  }
  digitalWrite(7,HIGH);  
  while(1){}
  
}
