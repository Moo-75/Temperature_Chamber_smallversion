#include <math.h>

#define THERMISTOR_PIN A0
#define IS_R_PIN A1
#define IS_L_PIN A2
#define TEMPERATURE_NOMINAL 25
#define THERMISTOR_NOMINAL 10000
#define NUM_SAMPLES 5
#define BCO_EFFICIENT 3950
#define SERIES_RESISTOR 10000

const int RPWM = 9;
const int LPWM = 10;
const int R_EN = 7;
const int L_EN = 8;
const int PWM_TOP = 3124;

const float DT_SEC = 0.10f;
const unsigned long interval = 100;

// Filter and spike rejection for rate of change (dT/dt)
const float SPIKE_C = 1.2f;
const float RATE_TAU_SEC = 0.8f;

// Robust PID with Derivative on Measurement (Damping) & Clamping Anti-Windup
// 1) KP = 0.25: ±4°C error provides 100% full saturation for fast transition.
// 2) KD = 1.60: Strong predictive damping against velocity (-KD * dT/dt) to eliminate overshoot.
// 3) KI = 0.025: Clean integral to eliminate steady-state error in < 8s.
// 4) INTEGRAL_ZONE_C = 1.2f: Integrator only activates within ±1.2°C of target to prevent windup during jumps.
// 5) RATE_GATE_C_PER_S = 0.13: Do not integrate while |dT/dt| is large, so approach
//    does not accumulate a heat/cool I bias that reheats after the target is passed.
const float KP = 0.25f;
const float KD = 1.60f;
const float KI = 0.025f;
const float INTEGRAL_ZONE_C = 1.2f;
const float RATE_GATE_C_PER_S = 0.13f;
const float I_MAX = 0.50f;

// Slew rate limit: output change cannot exceed ±15% per 100ms cycle (~0.67s for 0->100%).
const float DU_MAX = 0.15f;

float target_temperature = 25.0f;
bool is_running = true;

// Open-loop PWM probe (FORCE_PWM). Auto-expires so a hung serial session
// cannot leave the Peltier at full duty.
const unsigned long FORCE_MAX_MS = 4000;
unsigned long force_until_ms = 0;
float force_u = 0.0f;

unsigned long previousMillis = 0;

bool have_prev_temp = false;
float prev_temp = 0.0f;
float rate_c_per_s = 0.0f;
float i_term = 0.0f;
float u_applied = 0.0f;

float last_t = NAN;
float last_t_pred = NAN;
float last_u = 0.0f;
float last_d_term = 0.0f;
float last_adc = NAN;
float last_ohm = NAN;
int last_pwm = 0;

bool readFloatArgument(String command, float &value);
void handleSerialCommands();
void runTemperatureControl();
void applySignedDuty(float u);
void stopMotor();
void restoreDrivePins();
int probeHold(int pin);
int probePullup(int pin);
float analogMean(int pin, int n);
float measure_temp(int pin);
float clampf(float x, float lo, float hi);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10);
  ICR1 = PWM_TOP;

  Serial.println("Arduino Ready. Command-based control.");
}

void loop() {
  handleSerialCommands();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (force_until_ms != 0) {
      if ((long)(currentMillis - force_until_ms) >= 0) {
        force_until_ms = 0;
        force_u = 0.0f;
        stopMotor();
        u_applied = 0.0f;
        last_u = 0.0f;
        last_pwm = 0;
      } else {
        last_t = measure_temp(THERMISTOR_PIN);
        applySignedDuty(force_u);
        last_u = force_u;
        u_applied = force_u;
      }
    } else if (is_running) {
      runTemperatureControl();
    } else {
      stopMotor();
      u_applied = 0.0f;
      last_u = 0.0f;
      last_pwm = 0;
      rate_c_per_s *= 0.85f;
    }
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("GET_TEMP")) {
      float temp = measure_temp(THERMISTOR_PIN);
      Serial.println(temp, 4);
    } else if (command.startsWith("SET_TEMP")) {
      float parsed_temp;
      if (readFloatArgument(command, parsed_temp)) {
        if (fabs(parsed_temp - target_temperature) > 1.0f) {
          i_term = 0.0f;  // Clear stale integrator bias on setpoint jump
        }
        target_temperature = parsed_temp;
      } else {
        Serial.println("ERR");
      }
    } else if (command.equals("START")) {
      is_running = true;
    } else if (command.equals("STOP")) {
      is_running = false;
      force_until_ms = 0;
      force_u = 0.0f;
      stopMotor();
      u_applied = 0.0f;
      last_pwm = 0;
    } else if (command.startsWith("FORCE_PWM")) {
      float parsed_u;
      if (readFloatArgument(command, parsed_u)) {
        force_u = clampf(parsed_u, -1.0f, 1.0f);
        force_until_ms = millis() + FORCE_MAX_MS;
        applySignedDuty(force_u);
        last_u = force_u;
        u_applied = force_u;
      } else {
        Serial.println("ERR");
      }
    } else if (command.equals("GET_TARGET")) {
      Serial.println(target_temperature, 4);
    } else if (command.equals("GET_DRIVE")) {
      // running,force,ocr1a,ocr1b,ren,len,u,pwm,target,t
      Serial.print(is_running ? 1 : 0);
      Serial.print(',');
      Serial.print(force_until_ms != 0 ? 1 : 0);
      Serial.print(',');
      Serial.print((int)OCR1A);
      Serial.print(',');
      Serial.print((int)OCR1B);
      Serial.print(',');
      Serial.print(digitalRead(R_EN));
      Serial.print(',');
      Serial.print(digitalRead(L_EN));
      Serial.print(',');
      Serial.print(last_u, 3);
      Serial.print(',');
      Serial.print(last_pwm);
      Serial.print(',');
      Serial.print(target_temperature, 3);
      Serial.print(',');
      Serial.println(last_t, 3);
    } else if (command.equals("PIN_PROBE")) {
      // Detect IBT-2 input pulldowns on D7/D8/D9/D10 without extra jumpers.
      // hold=0 means the pin discharges (wired to a pulldown). hold=1 is floating.
      force_until_ms = 0;
      force_u = 0.0f;
      is_running = false;
      stopMotor();
      TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0));
      int h7 = probeHold(R_EN);
      int p7 = probePullup(R_EN);
      int h8 = probeHold(L_EN);
      int p8 = probePullup(L_EN);
      int h9 = probeHold(RPWM);
      int p9 = probePullup(RPWM);
      int h10 = probeHold(LPWM);
      int p10 = probePullup(LPWM);
      restoreDrivePins();
      Serial.print(h7);
      Serial.print(',');
      Serial.print(p7);
      Serial.print(',');
      Serial.print(h8);
      Serial.print(',');
      Serial.print(p8);
      Serial.print(',');
      Serial.print(h9);
      Serial.print(',');
      Serial.print(p9);
      Serial.print(',');
      Serial.print(h10);
      Serial.print(',');
      Serial.println(p10);
    } else if (command.equals("GET_IS")) {
      Serial.print(analogMean(IS_R_PIN, 8), 1);
      Serial.print(',');
      Serial.println(analogMean(IS_L_PIN, 8), 1);
    } else if (command.equals("GET_CTRL")) {
      // t,t_pred,rate,u,d_term,i_term,target,adc,ohm,pwm
      Serial.print(last_t, 3);
      Serial.print(',');
      Serial.print(last_t_pred, 3);
      Serial.print(',');
      Serial.print(rate_c_per_s, 4);
      Serial.print(',');
      Serial.print(last_u, 3);
      Serial.print(',');
      Serial.print(last_d_term, 4);
      Serial.print(',');
      Serial.print(i_term, 3);
      Serial.print(',');
      Serial.print(target_temperature, 3);
      Serial.print(',');
      Serial.print(last_adc, 1);
      Serial.print(',');
      Serial.print(last_ohm, 1);
      Serial.print(',');
      Serial.println(last_pwm);
    }
  }
}

bool readFloatArgument(String command, float &value) {
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) {
    return false;
  }
  String token = command.substring(commaIndex + 1);
  token.trim();
  if (token.length() == 0) {
    return false;
  }
  bool saw_digit = false;
  bool saw_dot = false;
  for (unsigned int i = 0; i < token.length(); i++) {
    char c = token.charAt(i);
    if (isDigit(c)) {
      saw_digit = true;
    } else if (c == '.' && !saw_dot) {
      saw_dot = true;
    } else if (c == '-' && i == 0) {
      continue;
    } else {
      return false;
    }
  }
  if (!saw_digit) {
    return false;
  }
  value = token.toFloat();
  return value > -50.0f && value < 100.0f;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void runTemperatureControl() {
  float curr = measure_temp(THERMISTOR_PIN);
  last_t = curr;
  if (isnan(curr)) {
    stopMotor();
    u_applied = 0.0f;
    last_u = 0.0f;
    last_pwm = 0;
    last_t_pred = NAN;
    return;
  }

  // 1) Filter temperature rate of change (dT/dt)
  if (!have_prev_temp) {
    prev_temp = curr;
    have_prev_temp = true;
  } else {
    float dT = curr - prev_temp;
    if (fabs(dT) > SPIKE_C) {
      prev_temp = curr;
    } else {
      float rate_raw = dT / DT_SEC;
      float alpha = DT_SEC / (RATE_TAU_SEC + DT_SEC);
      rate_c_per_s += alpha * (rate_raw - rate_c_per_s);
      prev_temp = curr;
    }
  }

  last_t_pred = curr + 3.0f * rate_c_per_s;

  // 2) Standard PID with Derivative on Measurement (Damping)
  // Error: positive = too cold (needs heating), negative = too hot (needs cooling)
  float error = target_temperature - curr;
  float p_term = KP * error;
  float d_term = -KD * rate_c_per_s;
  last_d_term = d_term;

  float u_unsat = p_term + d_term + i_term;
  float u = clampf(u_unsat, -1.0f, 1.0f);

  // 3) Conditional Integration (zone + anti-windup + rate gate)
  // Skip I while moving fast so a bump does not bank heat/cool bias that
  // turns PWM back on after the sensor crosses the target.
  bool in_integral_zone = (fabs(error) <= INTEGRAL_ZONE_C);
  bool rate_gated = (fabs(rate_c_per_s) > RATE_GATE_C_PER_S);
  bool saturating_high = (u_unsat >= 1.0f) && (error > 0.0f);
  bool saturating_low  = (u_unsat <= -1.0f) && (error < 0.0f);
  if (in_integral_zone && !rate_gated && !saturating_high && !saturating_low) {
    i_term += DT_SEC * (KI * error);
    i_term = clampf(i_term, -I_MAX, I_MAX);
  }

  // 4) Rate limiter on final duty cycle
  float du = clampf(u - u_applied, -DU_MAX, DU_MAX);
  u = u_applied + du;

  applySignedDuty(u);
  u_applied = u;
  last_u = u;
}

void applySignedDuty(float u) {
  if (fabs(u) < 0.02f) {
    stopMotor();
    last_pwm = 0;
    return;
  }
  int ticks = (int)(fabs(u) * (float)PWM_TOP + 0.5f);
  if (ticks < 1) {
    stopMotor();
    last_pwm = 0;
    return;
  }
  if (ticks > PWM_TOP) {
    ticks = PWM_TOP;
  }
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  // D9 (OCR1A) is Heating, D10 (OCR1B) is Cooling
  if (u > 0.0f) {
    OCR1A = ticks;
    OCR1B = 0;
    last_pwm = ticks;
  } else {
    OCR1A = 0;
    OCR1B = ticks;
    last_pwm = -ticks;
  }
}

void stopMotor() {
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  OCR1A = 0;
  OCR1B = 0;
}

void restoreDrivePins() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  OCR1A = 0;
  OCR1B = 0;
}

int probeHold(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(2);
  pinMode(pin, INPUT);
  delayMicroseconds(250);
  return digitalRead(pin);
}

int probePullup(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(2);
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(250);
  return digitalRead(pin);
}

float analogMean(int pin, int n) {
  long acc = 0;
  for (int i = 0; i < n; i++) {
    acc += analogRead(pin);
    delayMicroseconds(200);
  }
  return (float)acc / (float)n;
}

float measure_temp(int pin) {
  uint16_t samples[NUM_SAMPLES];
  float total = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(100);
  }
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += samples[i];
  }

  float average_adc = total / NUM_SAMPLES;
  last_adc = average_adc;
  if (average_adc < 1.0f || average_adc > 1022.0f) {
    last_ohm = NAN;
    return NAN;
  }
  float resistance = SERIES_RESISTOR / (1023.0f / average_adc - 1.0f);
  last_ohm = resistance;
  if (resistance <= 0.0f) {
    return NAN;
  }
  float steinhart = log(resistance / THERMISTOR_NOMINAL) / BCO_EFFICIENT;
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
  return 1.0f / steinhart - 273.15f;
}
