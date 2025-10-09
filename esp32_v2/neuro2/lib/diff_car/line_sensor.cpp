#include "diff_car.h"
#include <cmath>
#include <Preferences.h>



void DiffCar::setup_line_sensor(){
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){(uint8_t)SEN_REF_D1, (uint8_t)SEN_REF_D2, (uint8_t)SEN_REF_D3, (uint8_t)SEN_REF_D4, (uint8_t)SEN_REF_D5, (uint8_t)SEN_REF_D6, (uint8_t)SEN_REF_D7, (uint8_t)SEN_REF_D8}, 8);
    delay(100);
    // Try to load existing calibration from NVS
    if (this->load_qtr_calibration()) {
        Serial.println("Loaded QTR calibration from NVS.");
        this->calibrated = true;
    } else {
        if (!this->calibrated) {
        calibrate_line_sensors();
        }
    }
}

int16_t DiffCar::line_dist_center_mm() {
    float line_position = (float)this->line_position_value / 1000.0f;
    return (int16_t)(line_position* 100.0f - 350.0f);  // Converter para mm e centralizar em zero
}

void DiffCar::calibrate_line_sensors() {
    Serial.print("Calibrating...");
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
        
        if (i % 10 == 0) {
            vTaskDelay(10 / portTICK_PERIOD_MS);  // 10 ms
        }
        yield();  // alimenta o watchdog e BLE
    }
    Serial.println(" Done!");
    this->calibrated = true;
    this->save_qtr_calibration();   
}

void DiffCar::update_line_position() {
    uint16_t line_position = qtr.readLineBlack(this -> line_sensor_array);
    int16_t xcentro_mm = line_dist_center_mm();  // Usar versão inteira
    this->line_distance_mm = xcentro_mm;
    this->light_sensor.orientation.z = atan2(xcentro_mm, 15000);
    this->line_position_value = line_position;
}

void DiffCar::debug_line() {
    Serial.print("Line sensor readings: ");
    for (int i = 0; i < 8; i++) {
        Serial.print(this->line_sensor_array[i],4);
        if (i < 7) Serial.print(", ");
    }
    
    Serial.print(" | Position: ");
    Serial.print(this->line_position_value);
    Serial.print(" | Dist from center (mm): ");
    Serial.print(this->line_distance_mm); 
    Serial.print(" | Angle (rad): ");
    Serial.print(this->light_sensor.orientation.z);
    Serial.println();
}

void DiffCar::follow_line(float base_speed, float kp) {
    // Detecta marcadores de linha (linhas perpendiculares)
    this->detect_line_markers();
		
    float error = (float)this->line_distance_mm; 
    // Correção proporcional
    float correction = kp * error;
    
    // Limita a correção para evitar mudanças bruscas
    const float MAX_CORRECTION = 0.3;
    if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
    if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
    
    // Calcula velocidades dos motores
    // Se erro > 0 (linha à direita), motor direito mais lento, esquerdo mais rápido
    // Se erro < 0 (linha à esquerda), motor esquerdo mais lento, direito mais rápido
    float left_speed = base_speed + correction;
    float right_speed = base_speed - correction;
    
    // Garante que velocidades estejam dentro dos limites
    const float MAX_SPEED = 0.5;
    const float MIN_SPEED = -0.1;  // Permite pequena reversão para curvas acentuadas
    
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    
    // Define velocidades alvo
    this->left_velocity_target = left_speed;
    this->right_velocity_target = right_speed;
}

void DiffCar::detect_line_markers() {
    // Detecta marcadores de linha (linhas perpendiculares à linha principal)
    // Quando todos os 8 sensores detectam preto simultaneamente = marcador
    uint8_t sensors_active = 0;
    uint32_t total_value = 0;
    
    for (int i = 0; i < 8; i++) {
        total_value += this->line_sensor_array[i];
        // Se valor alto = preto detectado
        if (this->line_sensor_array[i] > 2000) {
            sensors_active++;
        }
    }
    
    // Média dos sensores para detecção mais robusta
    uint16_t avg_value = total_value / 8;
    bool marker_now = (sensors_active >= 6) || (avg_value > this->marker_threshold);
    
    // Detecção de borda (transição de não-marcador para marcador)
    if (marker_now && !this->line_marker_detected) {
        // Marcador detectado! Incrementa contador
        this->line_marker_count++;
        // Calcula distância baseada no número de marcadores
        this->line_marker_distance_m = (float)this->line_marker_count * this->marker_spacing_m;
        // Atualiza timestamp
        this->last_marker_time_ms = millis();
        // Debug
        Serial.printf("Marcador #%d detectado! Distância: %.2f m\n", 
                     this->line_marker_count, this->line_marker_distance_m);
    }
    this->line_marker_detected = marker_now;
}

void DiffCar::reset_line_markers() {
    // Reseta contador de marcadores (útil ao iniciar novo percurso)
    this->line_marker_count = 0;
    this->line_marker_distance_m = 0.0;
    this->line_marker_detected = false;
    this->last_marker_time_ms = millis();
}

// --- QTR calibration persistence -------------------------------------------
bool DiffCar::load_qtr_calibration() {
    Preferences prefs;
    if (!prefs.begin("diffcar", true)) { // read-only
        return false;
    }
    // Check version/key existence
    if (!prefs.isKey("qtr_min") || !prefs.isKey("qtr_max")) {
        prefs.end();
        return false;
    }
    // IMPORTANT: The QTR library does not allocate memory for calibration
    // values until calibrate() is called. We must call it at least once to
    // allocate the arrays before we try to load data into them.
    qtr.calibrate();
    size_t expect = sizeof(uint16_t) * 8;
    size_t got_min = prefs.getBytesLength("qtr_min");
    size_t got_max = prefs.getBytesLength("qtr_max");
    if (got_min != expect || got_max != expect) {
        Serial.printf("[QTR] Invalid calibration sizes (min=%u max=%u expected=%u)\n",
                      got_min, got_max, expect);
        prefs.end();
        return false;
    }
    uint16_t buf_min[8];
    uint16_t buf_max[8];
    prefs.getBytes("qtr_min", buf_min, expect);
    prefs.getBytes("qtr_max", buf_max, expect);
    for (uint8_t i = 0; i < 8; i++) {
        qtr.calibrationOn.minimum[i] = buf_min[i];
        qtr.calibrationOn.maximum[i] = buf_max[i];
    }
    prefs.end();
    Serial.println("[QTR] Calibration loaded successfully");
    return true;
}

void DiffCar::save_qtr_calibration() {
    Preferences prefs;
    if (!prefs.begin("diffcar", false)) { // read-write
        Serial.println("[QTR] Failed to open preferences for writing");
        return;
    }
    uint16_t buf_min[8];
    uint16_t buf_max[8];
    for (uint8_t i = 0; i < 8; i++) {
        buf_min[i] = qtr.calibrationOn.minimum[i];
        buf_max[i] = qtr.calibrationOn.maximum[i];
    }
    size_t written_min = prefs.putBytes("qtr_min", buf_min, sizeof(buf_min));
    size_t written_max = prefs.putBytes("qtr_max", buf_max, sizeof(buf_max));
    prefs.end();
    Serial.printf("[QTR] Calibration saved (%u/%u bytes)\n", written_min, written_max);
}

