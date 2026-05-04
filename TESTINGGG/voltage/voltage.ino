/*
DC Voltmeter Using a Voltage Divider (ESP32 Version)
*/

int analogInput = 34;  // Use GPIO34 (ADC1, safe input pin)

float vout = 0.0;
float vin = 0.0;

float R1 = 30000.0; // 30k
float R2 = 7500.0;  // 7.5k

int value = 0;

void setup() {
  pinMode(analogInput, INPUT);
  Serial.begin(115200);
  Serial.println("DC VOLTMETER");
}

void loop() {
  // Read ADC value (0–4095)
  value = analogRead(analogInput);

  // Convert to voltage (ESP32 uses 3.3V reference)
  vout = (value * 3.3) / 4095.0;

  // Calculate input voltage using voltage divider formula
  vin = vout / (R2 / (R1 + R2));

  Serial.print("INPUT V = ");
  Serial.println(vin, 2);

  delay(500);
}