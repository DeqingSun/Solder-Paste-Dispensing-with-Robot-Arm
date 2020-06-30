String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

unsigned long previousMillis = 0;
unsigned long laserStartMillis = 0;
unsigned long touchpadStartMillis = 0;
bool laserDetecting = false;
bool touchpadDetecting = false;

#define LASER_TIMEOUT  30000
#define TOUCHPAD_TIMEOUT  30000

#define LASER_ON {digitalWrite(5, HIGH);digitalWrite(6, HIGH);}
#define LASER_DIM {analogWrite(5, 5);analogWrite(6, 5);}

void setup() {
  // put your setup code here, to run once:
  LASER_DIM;
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(8, INPUT);
  digitalWrite(8, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  if (laserDetecting) {
    if ((signed long)(currentMillis - previousMillis) >= 10) {
      previousMillis = currentMillis;
      Serial.print(analogRead(3));
      Serial.print(',');
      Serial.println(analogRead(2));
      if ((signed long)(currentMillis - laserStartMillis) >= LASER_TIMEOUT) {
        LASER_DIM;
        laserDetecting = false;
      }
    }
  }

  if (touchpadDetecting) {
    if ((signed long)(currentMillis - previousMillis) >= 10) {
      previousMillis = currentMillis;
      Serial.println(digitalRead(8));
      if ((signed long)(currentMillis - touchpadStartMillis) >= TOUCHPAD_TIMEOUT) {
        touchpadDetecting = false;
      }
    }
  }


  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      break;
    }
  }

  if (stringComplete) {
    inputString.trim();
    char firstChar = inputString.charAt(0);
    if (firstChar == 'L') {
      LASER_ON;
      laserStartMillis = millis();
      touchpadDetecting = false;
      laserDetecting = true;
    } else if (firstChar == 'T') {
      touchpadStartMillis = millis();
      laserDetecting = false;
      touchpadDetecting = true;
    } else if (firstChar == 'S') {
      LASER_DIM;
      laserDetecting = false;
      touchpadDetecting = false;
    }
    inputString = "";
  }

}
