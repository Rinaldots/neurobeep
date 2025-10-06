#include "diff_car.h"

#ifdef ENCODER_QUAD

#include "ESP32Encoder.h"

ESP32Encoder encoder_left;
ESP32Encoder encoder_right;

void DiffCar::setup_encoder(){
    encoder_left.attachFullQuad(ENCODER_A_1, ENCODER_A_2);
    encoder_right.attachFullQuad(ENCODER_B_1, ENCODER_B_2);
    encoder_left.setCount(0);
    encoder_right.setCount(0);
    encoder_left.resumeCount();
    encoder_right.resumeCount();
}
void DiffCar::debug_encoder(){
    Serial.println("Encoder count = " + String((int32_t)encoder_left.getCount()) + " " + String((int32_t)encoder_right.getCount()));
    //Serial.print("encoder Left: "); Serial.print(diffCar.left_velocity_ms);
    Serial.print("Left Velocity (m/s): "); Serial.print(left_velocity_ms);
    //Serial.print("| encoder Right: "); Serial.print(diffCar.right_velocity_ms);
    Serial.print("Right Velocity (m/s): "); Serial.println(right_velocity_ms);
    Serial.print("Left Motor PWM: "); Serial.print(left_motor_pwm);
    Serial.print(" | Right Motor PWM: "); Serial.print(right_motor_pwm);
    Serial.print(" | Left Gain: "); Serial.print(left_gain);
    Serial.print(" | Right Gain: "); Serial.print(right_gain);
    Serial.print(" | Left Target (m/s): "); Serial.print(left_velocity_target);
    Serial.print(" | Right Target (m/s): "); Serial.print(right_velocity_target);
    Serial.println();
 }

void DiffCar::velocity_update(){
    unsigned long now_ms = millis();
    if ((now_ms - last_sample_time_ms) >= SAMPLE_MS) {
        int64_t left_count_now = encoder_left.getCount();
        int64_t right_count_now = encoder_right.getCount();
        unsigned long dt_ms = now_ms - last_sample_time_ms;
        long delta_left = left_count_now - last_count_left_snapshot;
        long delta_right = right_count_now - last_count_right_snapshot;
        float left_freq_window = 0.0f;
        float right_freq_window = 0.0f;
        if (dt_ms > 0) {
            left_freq_window = (float)delta_left / (dt_ms / 1000.0f);   // pulsos/s
            right_freq_window = (float)delta_right / (dt_ms / 1000.0f); // pulsos/s
        }

        left_freq_filtered = EMA_ALPHA * left_freq_window + (1.0f - EMA_ALPHA) * left_freq_filtered;
        right_freq_filtered = EMA_ALPHA * right_freq_window + (1.0f - EMA_ALPHA) * right_freq_filtered;

        last_count_left_snapshot = left_count_now;
        last_count_right_snapshot = right_count_now;
        last_sample_time_ms = now_ms;
    }
    // converter frequência para velocidade linear
    left_velocity_ms = (left_freq_filtered / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;
    right_velocity_ms = (right_freq_filtered / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;
}


#endif



#ifdef ENCODER_SIMPLE

void IRAM_ATTR left_encoder_isr() {
  unsigned long now = micros();
  unsigned long prev = diffCar.left_last_pulse_us;
  // debounce por tempo mínimo entre pulsos
  if (prev == 0 || (now - prev) > DEBOUNCE_US) {
    diffCar.left_last_interval_us = now - prev; // 0 para primeiro pulso, trata depois
    diffCar.left_last_pulse_us = now;
    diffCar.encoder_left_count++;
  }
}

void IRAM_ATTR right_encoder_isr() {
  unsigned long now = micros();
  unsigned long prev = diffCar.right_last_pulse_us;
  if (prev == 0 || (now - prev) > DEBOUNCE_US) {
    diffCar.right_last_interval_us = now - prev;
    diffCar.right_last_pulse_us = now;
    diffCar.encoder_right_count++;
  }
}

void DiffCar::setup_encoder() {
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), right_encoder_isr, RISING);
  last_sample_time_ms = millis();
}

// função simples para mediana de 3 (ajusta se MEDIAN_WINDOW mudar)
unsigned long median3(unsigned long a, unsigned long b, unsigned long c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void DiffCar::velocity_update() {
  unsigned long now_ms = millis();

  noInterrupts();
  unsigned long left_last_interval_us = this->left_last_interval_us;
  unsigned long right_last_interval_us = this->right_last_interval_us;
  interrupts();

  // atualizar buffers de mediana
  if (left_last_interval_us > 0) {
    left_intervals[left_idx % MEDIAN_WINDOW] = left_last_interval_us;
    left_idx++;
  }
  if (right_last_interval_us > 0) {
    right_intervals[right_idx % MEDIAN_WINDOW] = right_last_interval_us;
    right_idx++;
  }

  // pega mediana (MEDIAN_WINDOW = 3 neste exemplo)
  unsigned long left_med = 0, right_med = 0;
  if (left_idx >= MEDIAN_WINDOW) {
    left_med = median3(left_intervals[0], left_intervals[1], left_intervals[2]);
  }
  if (right_idx >= MEDIAN_WINDOW) {
    right_med = median3(right_intervals[0], right_intervals[1], right_intervals[2]);
  }

  float left_freq_from_interval = 0.0f;  // pulsos por segundo
  float right_freq_from_interval = 0.0f;
  if (left_med > 0) left_freq_from_interval = 1e6f / (float)left_med;
  if (right_med > 0) right_freq_from_interval = 1e6f / (float)right_med;

  // --- 2) Método "contagem por janela" (bom em altas velocidades)
  if ((now_ms - last_sample_time_ms) >= SAMPLE_MS) {
    noInterrupts();
    long left_count_now = this->encoder_left_count;
    long right_count_now = this->encoder_right_count;
    interrupts();
    unsigned long dt_ms = now_ms - last_sample_time_ms;
    long delta_left = left_count_now - last_count_left_snapshot;
    long delta_right = right_count_now - last_count_right_snapshot;
    float left_freq_window = 0.0f;
    float right_freq_window = 0.0f;
    if (dt_ms > 0) {
      left_freq_window = (float)delta_left / (dt_ms / 1000.0f);   // pulsos/s
      right_freq_window = (float)delta_right / (dt_ms / 1000.0f); // pulsos/s
    }
    float left_freq_combined = 0.0f;
    float right_freq_combined = 0.0f;
    if (delta_left >= 3) left_freq_combined = left_freq_window;
    else left_freq_combined = left_freq_from_interval;
    if (delta_right >= 3) right_freq_combined = right_freq_window;
    else right_freq_combined = right_freq_from_interval;
    // --- EMA (suaviza)
    left_freq_filtered = EMA_ALPHA * left_freq_combined + (1.0f - EMA_ALPHA) * left_freq_filtered;
    right_freq_filtered = EMA_ALPHA * right_freq_combined + (1.0f - EMA_ALPHA) * right_freq_filtered;
    // snapshots para próxima janela
    last_count_left_snapshot = left_count_now;
    last_count_right_snapshot = right_count_now;
    last_sample_time_ms = now_ms;
  }
  // --- Conversões úteis (ex: RPM e velocidade linear)
  // Anti-drift: timeout sem novos pulsos
  if ((now_ms - (this->right_last_pulse_us / 1000UL)) > NO_PULSE_TIMEOUT_MS) {
    right_freq_filtered = 0.0f;
    right_stopped = true;
  } else {
    right_stopped = false;
  }
  if ((now_ms - (this->left_last_pulse_us / 1000UL)) > NO_PULSE_TIMEOUT_MS) {
    left_freq_filtered = 0.0f;
    left_stopped = true;
  } else {
    left_stopped = false;
  }
  // Deadband para valores muito baixos (ruído)
  if (right_freq_filtered < MIN_PULSES_PER_S) right_freq_filtered = 0.0f;
  if (left_freq_filtered  < MIN_PULSES_PER_S) left_freq_filtered  = 0.0f;
  float left_pulses_per_s = left_freq_filtered;
  float right_pulses_per_s = right_freq_filtered;
  float left_rpm = 0.0f, right_rpm = 0.0f;
  float left_m_s = 0.0f, right_m_s = 0.0f;
  if (pulses_per_rev > 0) {
    left_rpm = (left_pulses_per_s / pulses_per_rev) * 60.0f;
    right_rpm = (right_pulses_per_s / pulses_per_rev) * 60.0f;
    left_m_s = (PI * wheel_diameter_m / pulses_per_rev) * left_pulses_per_s;
    right_m_s = (PI * wheel_diameter_m / pulses_per_rev) * right_pulses_per_s;
  }
  left_velocity_ms = left_m_s;
  right_velocity_ms = right_m_s;
  
}

void DiffCar::debug_encoder() {
    Serial.print("encoder Left: "); Serial.print(diffCar.encoder_left_count);
    //Serial.print("Left Velocity (m/s): "); Serial.print(left_velocity_ms);
    Serial.print("| encoder Right: "); Serial.print(diffCar.encoder_right_count);
    //Serial.print("Right Velocity (m/s): "); Serial.println(right_velocity_ms);
    Serial.println();

}




#endif