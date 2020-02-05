

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial3.begin(9600);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  Serial3.println("Hello from the Arduino MEGA!");
}
