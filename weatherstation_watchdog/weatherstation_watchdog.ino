#define BC337_PIN 2

void setup() {
  // put your setup code here, to run once:
  pinMode(BC337_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(BC337_PIN, HIGH); // do this first
  delay(500);  
  digitalWrite(BC337_PIN, LOW); // do this first
    delay(2000);  
}
