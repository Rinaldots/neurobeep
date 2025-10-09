#include "diff_car.h"
#include <cmath>



void DiffCar::setup_line_sensor(){
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){(uint8_t)SEN_REF_D1, (uint8_t)SEN_REF_D2, (uint8_t)SEN_REF_D3, (uint8_t)SEN_REF_D4, (uint8_t)SEN_REF_D5, (uint8_t)SEN_REF_D6, (uint8_t)SEN_REF_D7, (uint8_t)SEN_REF_D8}, 8);

    delay(100);
    if (!this->calibrated) {
        Serial.print("Calibrating...");
        for (uint16_t i = 0; i < 400; i++)
        {
            qtr.calibrate();
        }
        Serial.println(" Done!");
        this->calibrated = true;
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(qtr.calibrationOn.minimum[i]);
            Serial.print(' ');
        }
        Serial.println();
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(qtr.calibrationOn.maximum[i]);
            Serial.print(' ');
        }
    }
    Serial.println();
    Serial.println();
    Serial.println("Line sensor setup complete.");
}

int16_t DiffCar::line_dist_center_mm() {
    float line_position = (float)this->line_position_value / 1000.0f;
    return (int16_t)(line_position* 100.0f - 350.0f);  // Converter para mm e centralizar em zero
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
		
    // Controlador PID simples para seguir linha
    // line_distance_mm: negativo = linha à esquerda, positivo = linha à direita
    
    // Calcula erro (distância da linha do centro em mm)
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
    
    // Calcula quantos sensores estão vendo preto
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
    
    // Detecta marcador: pelo menos 6 sensores ativos OU média acima do threshold
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
        Serial.printf("🎯 Marcador #%d detectado! Distância: %.2f m\n", 
                     this->line_marker_count, this->line_marker_distance_m);
    }
    
    // Atualiza flag de detecção para próxima iteração
    this->line_marker_detected = marker_now;
}

void DiffCar::reset_line_markers() {
    // Reseta contador de marcadores (útil ao iniciar novo percurso)
    this->line_marker_count = 0;
    this->line_marker_distance_m = 0.0;
    this->line_marker_detected = false;
    this->last_marker_time_ms = millis();
    Serial.println("📍 Marcadores resetados");
}
