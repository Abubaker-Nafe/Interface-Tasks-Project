const int trigPin = 11;
const int echoPin = 10;
const int potPin = A4;
const int greenLED = 3;
const int yellowLED = 4;
const int redLED = 5;

long duration;
int distance;
int minRange = 10;
int maxRange = 20;
int potValue;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  potValue = analogRead(potPin);
  int rangeAdjustment = map(potValue, 0, 1023, -5, 5);
  int adjustedMinRange = minRange + rangeAdjustment;
  int adjustedMaxRange = maxRange + rangeAdjustment;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Min Range: ");
  Serial.print(adjustedMinRange);
  Serial.print(" cm, Max Range: ");
  Serial.print(adjustedMaxRange);
  Serial.println(" cm");

  if (distance > adjustedMaxRange) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);
  } else if (distance <= adjustedMaxRange && distance > adjustedMinRange) {
    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, LOW);
  } else {
    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  delay(100);
}
