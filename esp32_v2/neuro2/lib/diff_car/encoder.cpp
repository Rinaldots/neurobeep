#include "diff_car.h"

#ifdef ENCODER_QUAD

#include "ESP32Encoder.h"

ESP32Encoder encoder_left;
ESP32Encoder encoder_right;

void DiffCar::setup_encoder(){
    encoder_left.attachHalfQuad(ENCODER_A_1, ENCODER_A_2);
    encoder_right.attachHalfQuad(ENCODER_B_1, ENCODER_B_2);
    encoder_left.setCount(0);
    encoder_right.setCount(0);
}
DiffCar::callback_encoder(){
    return encoder_left, encoder_right.read;
}
#endif

#ifdef ENCODER_SIMPLE

void IRAM_ATTR left_encoder_isr() {
    diffCar.encoder_left_time_now = millis();
    if (diffCar.encoder_left_time_now - diffCar.encoder_left_time_last > 50) {
        diffCar.encoder_left = diffCar.encoder_left + 1;
        diffCar.encoder_left_time_last = diffCar.encoder_left_time_now;
    }

}

void IRAM_ATTR right_encoder_isr() {
    diffCar.encoder_right_time_now = millis();
    if (diffCar.encoder_right_time_now - diffCar.encoder_right_time_last > 50) {
        diffCar.encoder_right = diffCar.encoder_right + 1;
        diffCar.encoder_right_time_last = diffCar.encoder_right_time_now;
    }
}

void DiffCar::setup_encoder() { 
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), left_encoder_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), right_encoder_isr, FALLING);
}

void DiffCar::velocity_update(){
    
    unsigned long current_time = millis();
    delta_time_velocity = (float)(current_time - cache_time_velocity);
    float delta_left_pass = (float)encoder_left - cache_left_pass;
    float delta_right_pass = (float)encoder_right - cache_right_pass;
    
    if(delta_time_velocity > 50.0f && delta_left_pass > 0.0f){
        // Check for division by zero
        if(delta_time_velocity > 0.0f) {
            left_velocity = delta_left_pass / delta_time_velocity;
            cache_left_pass = (float)encoder_left;
        } else {
            left_velocity = 0.0f;
        }
    }else{
        left_velocity = 0.0f;
    }
    
    if(delta_time_velocity > 50.0f && delta_right_pass > 0.0f){
        // Check for division by zero
        if(delta_time_velocity > 0.0f) {
            Serial.print("  delta");
            Serial.print(delta_right_pass);
            Serial.print("  time");
            Serial.println(delta_time_velocity);
            right_velocity = delta_right_pass / delta_time_velocity;
            cache_right_pass = (float)encoder_right;
        } else {
            right_velocity = 0.0f;
        }
    }else{
        right_velocity = 0.0f;
    }
    cache_time_velocity = current_time;
}

void DiffCar::debug_encoder(){
    Serial.print("L Enc: ");
    Serial.print(encoder_left);
    Serial.print("| L Vel: ");
    Serial.print(left_velocity);

    Serial.println();
    Serial.print("R Enc: ");
    Serial.print(encoder_right);
    Serial.print("| R Vel ");
    Serial.print(right_velocity);

    Serial.println();
}



#endif