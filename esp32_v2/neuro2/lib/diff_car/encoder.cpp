#ifdef ENCODER_QUAD

#include "ESP32Encoder.h"
void DiffCar::setup_encoder(){
    this->left_encoder_hall_pin_a = left_encoder_hall_pin_a;
    this->left_encoder_hall_pin_b = left_encoder_hall_pin_b;
    this->right_encoder_hall_pin_a = right_encoder_hall_pin_a;
    this->right_encoder_hall_pin_b = right_encoder_hall_pin_b;
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

void DiffCar::setup_encoder() {
    unsigned long left_encoder = 0;
    unsigned long right_encoder = 0;
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), left_encoder_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), right_encoder_isr, RISING);
}

void IRAM_ATTR left_encoder_isr() {
    encoder_left = encoder_left + 1;
}

void IRAM_ATTR right_encoder_isr() {
    encoder_right = encoder_right + 1;
}

void DiffCar::callback_encoder(){
    return encoder_left, encoder_right;
}
#endif