#include <math.h>

#define THERMISTOR_PIN A0
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

// Loop period (s). Keep in sync with `interval`.
const float DT_SEC = 0.10f;
const unsigned long interval = 100;

// Sensor-to-Peltier conduction lag (s). Cut power when T_pred hits target,
// not when the delayed sensor does. Geometry-dominated; not a PWM table.
const float TAU_SEC = 5.0f;

// Ignore tiny dT/dt so ADC noise does not move T_pred.
const float MIN_RATE_C_PER_S = 0.025f;

// Reject one-sample glitches (not a mouse sitting slowly on the sensor).
const float SPIKE_C = 1.2f;

// Rate EMA time constant (s). Smaller = noisier T_pred, larger = later landing.
const float RATE_TAU_SEC = 1.0f;

// Nominal dT/dt at |u|=1 near room temperature. Underestimate slightly.
// Heating vs cooling differ (Joule heat helps heat, fights cool).
const float B0_HEAT = 0.40f;
const float B0_COOL = 0.28f;

// Disturbance observer bandwidth (1/s) on residual: dT/dt - b0*u - f_hat.
// FAST during strong actuation so f_hat tracks leak as T itself moves
// (10 C vs 40 C). SLOW near hold so ADC/mouse jitter is not chased.
const float OBS_L_FAST = 1.6f;
const float OBS_L_SLOW = 0.55f;
const float OBS_F_MAX = 1.8f;

// P on predicted error. |e_pred| > 1/KP saturates -> full PWM (fast approach).
const float KP = 0.48f;

// Weak integral for leftover bias after the observer. Anti-windup via
// back-calculation so I does not store "100%" during a long approach.
const float KI = 0.04f;
const float TT_AW = 0.8f;
const float I_MAX = 0.85f;

float target_temperature = 25.0f;
bool is_running = true;

unsigned long previousMillis = 0;

bool have_prev_temp = false;
float prev_temp = 0.0f;
float rate_c_per_s = 0.0f;
float f_hat = 0.0f;
float i_term = 0.0f;
float u_applied = 0.0f;

float last_t = NAN;
float last_t_pred = NAN;
float last_u = 0.0f;

bool readFloatArgument(String command, float &value);
void handleSerialCommands();
void runTemperatureControl();
void applySignedDuty(float u);
void stopMotor();
float measure_temp(int pin);
float clampf(float x, float lo, float hi);
float b0_for_u(float u);

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
    if (is_running) {
      runTemperatureControl();
    } else {
      stopMotor();
      u_applied = 0.0f;
      last_u = 0.0f;
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
        target_temperature = parsed_temp;
        // Keep f_hat / i_term: they are the current thermal load, not the old target.
      } else {
        Serial.println("ERR");
      }
    } else if (command.equals("START")) {
      is_running = true;
    } else if (command.equals("STOP")) {
      is_running = false;
      stopMotor();
      u_applied = 0.0f;
    } else if (command.equals("GET_TARGET")) {
      Serial.println(target_temperature, 4);
    } else if (command.equals("GET_CTRL")) {
      // t,t_pred,rate,u,f_hat,i,target
      Serial.print(last_t, 3);
      Serial.print(',');
      Serial.print(last_t_pred, 3);
      Serial.print(',');
      Serial.print(rate_c_per_s, 4);
      Serial.print(',');
      Serial.print(last_u, 3);
      Serial.print(',');
      Serial.print(f_hat, 4);
      Serial.print(',');
      Serial.print(i_term, 3);
      Serial.print(',');
      Serial.println(target_temperature, 3);
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
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

float b0_for_u(float u) {
  return (u >= 0.0f) ? B0_HEAT : B0_COOL;
}

void runTemperatureControl() {
  float curr = measure_temp(THERMISTOR_PIN);
  last_t = curr;
  if (isnan(curr)) {
    stopMotor();
    u_applied = 0.0f;
    last_u = 0.0f;
    last_t_pred = NAN;
    return;
  }

  bool spike = false;
  if (!have_prev_temp) {
    prev_temp = curr;
    have_prev_temp = true;
  } else {
    float dT = curr - prev_temp;
    if (fabs(dT) > SPIKE_C) {
      spike = true;
      prev_temp = curr;
    } else {
      float rate_raw = dT / DT_SEC;
      float alpha = DT_SEC / (RATE_TAU_SEC + DT_SEC);
      rate_c_per_s += alpha * (rate_raw - rate_c_per_s);
      prev_temp = curr;
    }
  }

  float rate_for_pred = rate_c_per_s;
  if (fabs(rate_for_pred) < MIN_RATE_C_PER_S) {
    rate_for_pred = 0.0f;
  }
  float t_pred = curr + TAU_SEC * rate_for_pred;
  last_t_pred = t_pred;

  if (!spike) {
    float b0_app = b0_for_u(u_applied);
    float residual = rate_c_per_s - b0_app * u_applied - f_hat;
    float obs_l = (fabs(u_applied) > 0.70f) ? OBS_L_FAST : OBS_L_SLOW;
    f_hat += DT_SEC * obs_l * residual;
    f_hat = clampf(f_hat, -OBS_F_MAX, OBS_F_MAX);
  }

  float e_pred = target_temperature - t_pred;
  float b0_cmd = (e_pred >= 0.0f) ? B0_HEAT : B0_COOL;
  float u_unsat = (KP * e_pred + i_term - f_hat) / b0_cmd;
  float u = clampf(u_unsat, -1.0f, 1.0f);

  // Back-calculation: do not remember an impossible duty during saturation.
  i_term += DT_SEC * (KI * e_pred + (u - u_unsat) / TT_AW);
  i_term = clampf(i_term, -I_MAX, I_MAX);

  applySignedDuty(u);
  u_applied = u;
  last_u = u;
}

void applySignedDuty(float u) {
  if (fabs(u) < 0.02f) {
    stopMotor();
    return;
  }
  int ticks = (int)(fabs(u) * (float)PWM_TOP + 0.5f);
  if (ticks < 1) {
    stopMotor();
    return;
  }
  if (ticks > PWM_TOP) {
    ticks = PWM_TOP;
  }
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  // Existing B+/B- wiring heats on RPWM (D9/OCR1A) and cools on LPWM (D10/OCR1B).
  if (u > 0.0f) {
    OCR1A = ticks;
    OCR1B = 0;
  } else {
    OCR1A = 0;
    OCR1B = ticks;
  }
}

void stopMotor() {
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  OCR1A = 0;
  OCR1B = 0;
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
  if (average_adc < 1.0f || average_adc > 1022.0f) {
    return NAN;
  }
  // Same divider as Temperature_Chamber: NTC to GND, 10k to VCC.
  // R = SERIES / (1023/ADC - 1). The inverted form maps 20 C to ~30 C.
  float resistance = SERIES_RESISTOR / (1023.0f / average_adc - 1.0f);
  if (resistance <= 0.0f) {
    return NAN;
  }
  float steinhart = log(resistance / THERMISTOR_NOMINAL) / BCO_EFFICIENT;
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
  return 1.0f / steinhart - 273.15f;
}
