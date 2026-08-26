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

const float DT_SEC = 0.10f;
const unsigned long interval = 100;

// Filter and spike rejection for rate of change (dT/dt)
const float SPIKE_C = 1.2f;
const float RATE_TAU_SEC = 1.2f;

// Robust PID with Derivative on Measurement (Damping) & Clamping Anti-Windup
// 1) KP = 0.25: ±4°C error provides 100% full saturation for fast transition.
// 2) KD = 1.00: Smooth braking on velocity (-KD * dT/dt) to prevent overshoot.
// 3) KI = 0.025: Clean integral to eliminate steady-state error in < 8s.
// 4) Clamping Anti-windup: Freezes integrator during saturation to prevent windup.
const float KP = 0.25f;
const float KD = 1.00f;
const float KI = 0.025f;
const float I_MAX = 0.50f;

// Slew rate limit: output change cannot exceed ±15% per 100ms cycle (~0.67s for 0->100%).
const float DU_MAX = 0.15f;

float target_temperature = 25.0f;
bool is_running = true;

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
    if (is_running) {
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
      stopMotor();
      u_applied = 0.0f;
      last_pwm = 0;
    } else if (command.equals("GET_TARGET")) {
      Serial.println(target_temperature, 4);
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

  // 3) Conditional Integration (Clamping Anti-Windup)
  // Freeze integrator if output is saturated in the direction of error
  bool saturating_high = (u_unsat >= 1.0f) && (error > 0.0f);
  bool saturating_low  = (u_unsat <= -1.0f) && (error < 0.0f);
  if (!saturating_high && !saturating_low) {
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
