const byte bootPin = 2;
const byte interruptPin = 3;
volatile byte state = LOW;

void setup() {
  pinMode(bootPin, OUTPUT);
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), setBoot, RISING);
}

void loop() {
  ;
}

void setBoot() {
  state = !state;
  digitalWrite(bootPin, state);
}
