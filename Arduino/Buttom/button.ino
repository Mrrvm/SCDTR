const int interruptPin = 2;
bool state=0;
unsigned long last_interrupt=0;

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  Serial.println(state);
  delay(20);
}

void blink() {
  unsigned long now=millis();
  if(now-last_interrupt>200){
    state = !state;
    last_interrupt=now;
  }
}