void setup() {
  // Initialize serial communication at a baud rate of 9600
  Serial.begin(9600);

  // Initialize A2 and A3 as analog input pins
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
}

void loop() {
  // Read analog values from A2 and A3
  int analogValueA1 = analogRead(1);
  int analogValueA2 = analogRead(2);
  int analogValueA3 = analogRead(3);
  int analogValueA4 = analogRead(4);
  int analogValueA5 = analogRead(5);
  int analogValueA6 = analogRead(6);

  // Print the values to the serial monitor
  Serial.print("A1: ");
  Serial.print(analogValueA1);
  Serial.print("\t A2: ");
  Serial.print(analogValueA2);
  Serial.print("\t A3: ");
  Serial.print(analogValueA3);
  Serial.print("\t A4: ");
  Serial.print(analogValueA4);
  Serial.print("\t A5: ");
  Serial.print(analogValueA5);
  Serial.print("\tA6: ");
  Serial.println(analogValueA6);
  delay(100);
}
