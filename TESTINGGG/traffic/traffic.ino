int redPin = 25;
int yellowPin = 26;
int greenPin = 27;

char command;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  Serial.begin(115200);
  Serial.println("Send: R, Y, G, O");
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();

    // Ignore newline / carriage return
    if (command == '\n' || command == '\r') {
      return;
    }

    // Turn all OFF first
    digitalWrite(redPin, LOW);
    digitalWrite(yellowPin, LOW);
    digitalWrite(greenPin, LOW);

    if (command == 'R' || command == 'r') {
      digitalWrite(redPin, HIGH);
      Serial.println("RED ON");
    }
    else if (command == 'Y' || command == 'y') {
      digitalWrite(yellowPin, HIGH);
      Serial.println("YELLOW ON");
    }
    else if (command == 'G' || command == 'g') {
      digitalWrite(greenPin, HIGH);
      Serial.println("GREEN ON");
    }
    else if (command == 'O' || command == 'o') {
      Serial.println("ALL OFF");
    }
    else {
      Serial.println("Invalid command");
    }
  }
}