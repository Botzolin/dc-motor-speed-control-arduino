volatile int pulses = 0;

const int   RPWM = 5;
const int   LPWM = 6;
const int   ENC  = 2;

const float PPR       = 293.0;
const int   SAMPLE_MS = 50;

const float RPM_MIN   = 0.0;
const float RPM_MAX   = 200.0;
const float PI_B0     =  1.896;
const float PI_B1     = -1.137;
const int   PWM_MAX   = 255;
const int   PWM_MIN   = 0;

volatile float rpm_ref = 0.0;

float u_prev = 0.0;
float e_prev = 0.0;

////////////////////////////////////////////////////////////////////

void countPulse() { pulses++; }

void readSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    float val = input.toFloat();
    if      (val > RPM_MAX) val = RPM_MAX;
    else if (val < RPM_MIN) val = RPM_MIN;
    rpm_ref = val;
  }
}

////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(ENC,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC), countPulse, RISING);

  Serial.begin(115200);
  delay(1000);
}

void loop() {
  static unsigned long lastSample = millis();
  static unsigned long startTime  = millis();

  readSerial();

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    float dt = (now - lastSample) / 1000.0;
    lastSample = now;

    noInterrupts();
    int count = pulses;
    pulses = 0;
    interrupts();

    float rpm = (count / PPR) * (60.0 / dt);
    float e   = rpm_ref - rpm;
    float u   = u_prev + PI_B0 * e + PI_B1 * e_prev;

    if      (u > PWM_MAX) u = PWM_MAX;
    else if (u < PWM_MIN) u = PWM_MIN;

    analogWrite(RPWM, (int)u);
    analogWrite(LPWM, 0);

    u_prev = u;
    e_prev = e;

    // Trimite doar: timp, referinta, rpm
    float timeSec = (now - startTime) / 1000.0;
    Serial.print(timeSec, 2);  Serial.print(",");
    Serial.print(rpm_ref, 1);  Serial.print(",");
    Serial.println(rpm, 2);
  }
}