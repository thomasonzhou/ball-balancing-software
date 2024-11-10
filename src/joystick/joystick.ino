#define VRX_PIN A0
#define VRY_PIN A1
#define MAX_OUT 1.0
#define JOYSTICK_MIN 0.0
#define JOYSTICK_MAX 1023.0

#define X_DRIFT


const double JOYSTICK_MID = JOYSTICK_MAX / 2.0;
double xValue = 0; 
double yValue = 0; 

void setup() {
  Serial.begin(115200);
}

double get_value(int pin){
  int value = analogRead(pin);
  return (value - JOYSTICK_MID) / JOYSTICK_MID;
}

void loop() {
  xValue = get_value(VRX_PIN);
  yValue = get_value(VRY_PIN);

  Serial.print("<a");
  for (int i = 0; i < 3; i++){
    Serial.print(",");
    Serial.print(xValue);
  }
  Serial.println(">");

  delay(200);
}
