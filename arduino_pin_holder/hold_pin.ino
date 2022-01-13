const byte bootPin = 3;
const byte interruptPin = 2;
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
