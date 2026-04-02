// ---------------- PINI ----------------
const int IN1 = 5;
const int IN2 = 6;
const int ENA = 9;      // PWM spre L298N

const int SENSOR = 4;   // senzor IR pentru impulsuri
const int CLK = 10;     // encoder knob
const int DT  = 11;

// ---------------- SETARI ----------------
const int impulsuriPeRotatie = 8;          
const unsigned long Ts = 100;              
float Kp = 1.5;
float Ki = 0.8;

// ---------------- VARIABILE ----------------
int pwm = 150;                             // comanda initiala
int rpmDorit = 60;                         // setpoint din encoder knob

int impulsuri = 0;                         // impulsuri numarate in fereastra curenta
int ultimaStareSenzor = 0;
int ultimaStareCLK = 0;

float rpm = 0;
float rpmFiltrat = 0;
float sumaErori = 0;

unsigned long timpAnterior = 0;

void setup() {
  Serial.begin(9600);

  pinMode(SENSOR, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // motorul merge intr-un singur sens
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, pwm);

  ultimaStareCLK = digitalRead(CLK);
  ultimaStareSenzor = digitalRead(SENSOR);
  timpAnterior = millis();
}

void loop() {
  citesteKnob();

  unsigned long timpCurent = millis();

  // controlul ruleaza o data la 100 ms
  if (timpCurent - timpAnterior >= Ts) {
    float dt = (timpCurent - timpAnterior) / 1000.0;

    // calcul RPM din impulsurile adunate in fereastra Ts
    rpm = (impulsuri * 60.0) / (impulsuriPeRotatie * dt);

    // filtru simplu ca sa nu sara prea tare valorile
    rpmFiltrat = 0.7 * rpmFiltrat + 0.3 * rpm;

    // resetez impulsurile pentru fereastra urmatoare
    impulsuri = 0;

    // -------- CONTROL PI --------
    float eroare = rpmDorit - rpmFiltrat;

    // partea integrala
    sumaErori += eroare * dt;

    // limitare integrala
    if (sumaErori > 50) sumaErori = 50;
    if (sumaErori < -50) sumaErori = -50;

    // iesire PI
    float comanda = Kp * eroare + Ki * sumaErori;

    // actualizare PWM
    pwm += (int)comanda;

    // limitare PWM
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    analogWrite(ENA, pwm);

    Serial.print("RPM dorit = ");
    Serial.print(rpmDorit);
    Serial.print(" | RPM = ");
    Serial.print(rpmFiltrat);
    Serial.print(" | PWM = ");
    Serial.println(pwm);

    timpAnterior = timpCurent;
  }

  // numar impulsurile de la senzorul IR
  citesteSenzor();
}

// --------------------------------------------------
// Encoder knob -> modifica rpmDorit
// --------------------------------------------------
void citesteKnob() {
  int stareCLK = digitalRead(CLK);

  if (stareCLK != ultimaStareCLK) {
    if (digitalRead(DT) != stareCLK) {
      rpmDorit += 5;
    } else {
      rpmDorit -= 5;
    }

    if (rpmDorit < 0) rpmDorit = 0;
    if (rpmDorit > 200) rpmDorit = 200;
  }

  ultimaStareCLK = stareCLK;
}

// --------------------------------------------------
// Senzor IR -> numara impulsuri
// --------------------------------------------------
void citesteSenzor() {
  int stareSenzor = digitalRead(SENSOR);

  if (stareSenzor == 1 && ultimaStareSenzor == 0) {
    impulsuri++;
  }

  ultimaStareSenzor = stareSenzor;
}