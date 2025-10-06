// Include DiffCar class definition
#include "diff_car.h"  
#include <cmath>
QuickPID left_pid(&diffCar.left_velocity_ms, &diffCar.left_gain, &diffCar.left_velocity_target);
QuickPID right_pid(&diffCar.right_velocity_ms, &diffCar.right_gain, &diffCar.right_velocity_target);

void DiffCar::setup_h_bridge(){
    pinMode(MOTOR_EN_A, OUTPUT);
    pinMode(MOTOR_EN_B, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    ledcAttachChannel(MOTOR_EN_A, 30000, 8, 0);
    ledcAttachChannel(MOTOR_EN_B, 30000, 8, 1);
    left_pid.SetOutputLimits(-255, 255);
    left_pid.SetSampleTimeUs(100000);
    left_pid.SetTunings(KP, KI, KD, left_pid.pMode::pOnError, left_pid.dMode::dOnMeas, left_pid.iAwMode::iAwClamp);
    left_pid.SetMode(left_pid.Control::automatic);
    right_pid.SetOutputLimits(-255, 255);
    right_pid.SetSampleTimeUs(100000);
    right_pid.SetTunings(KP, KI, KD, right_pid.pMode::pOnError, right_pid.dMode::dOnMeas, right_pid.iAwMode::iAwClamp);
    right_pid.SetMode(right_pid.Control::automatic);
}

void DiffCar::set_motor_speed(float left_motor_pwm_in, float right_motor_pwm_in){
    left_motor_dir = 1;
    right_motor_dir = 1; 
    if (left_motor_pwm_in < 255 && left_motor_pwm_in > 0) left_motor_pwm = abs(left_motor_pwm_in);
    if (right_motor_pwm_in < 255 && right_motor_pwm_in > 0) right_motor_pwm = abs(right_motor_pwm_in);
}

static inline double model_velocity_from_pwm(double p){
    return (C1 * p * p) + (C2 * p) + C3;
}

static double pwm_from_velocity(double vel) {
    
    if (!std::isfinite(vel) || vel <= 0.0) return 0.0;
    const double vel_max = model_velocity_from_pwm(255.0);
    if (vel >= vel_max) return 255.0;
    const double MIN_EFFECTIVE_VEL = 0.05; 
    if (vel < MIN_EFFECTIVE_VEL) return 0.0;
    const double a = C1;
    const double b = C2;
    const double c = (C3 - vel);
    if (std::fabs(a) < 1e-9) { 
        double p = (vel - C3) / b;
        if (!std::isfinite(p)) return 0.0;
        if (p < 0) p = 0; else if (p > 255) p = 255;
        return p;
    }
    double disc = b*b - 4*a*c;
    if (disc < 0) {
        return 0.0;
    }
    double sqrt_disc = std::sqrt(disc);
    double denom = 2*a;
    double r1 = (-b + sqrt_disc) / denom;
    double r2 = (-b - sqrt_disc) / denom;
    double best = -1;
    double best_err = 1e12;
    auto try_root = [&](double r){
        if (!std::isfinite(r)) return; 
        if (r < 0 || r > 255) return; 
        double v_pred = model_velocity_from_pwm(r);
        double err = std::fabs(v_pred - vel);
        if (err < best_err) { best_err = err; best = r; }
    };
    try_root(r1);
    try_root(r2);
    if (best < 0) {
        if (r1 >= 0 && r1 < r2) best = r1; else best = r2;
        if (best < 0) best = 0; if (best > 255) best = 255;
    }
    return best;
}


MotorPwmResult DiffCar::set_motor_speed_msr(float vel_left, float vel_right){
    MotorPwmResult result;
    const float MAX_REASONABLE_VEL = 3.0f;
    if (fabs(vel_left) > MAX_REASONABLE_VEL || fabs(vel_right) > MAX_REASONABLE_VEL) {
        Serial.println("[WARN] Valores passados a set_motor_speed_ms parecem PWM. Use set_motor_speed().");
        left_motor_pwm = (int)std::clamp(fabs(vel_left), 0.0f, 255.0f);
        right_motor_pwm = (int)std::clamp(fabs(vel_right), 0.0f, 255.0f);
        left_motor_dir = vel_left >= 0 ? 1 : 0;
        right_motor_dir = vel_right >= 0 ? 1 : 0;
        result.left = left_motor_pwm;
        result.right = right_motor_pwm;
        return result;
    }
    double p_left = pwm_from_velocity(vel_left);
    double p_right = pwm_from_velocity(vel_right);
    if (!std::isfinite(p_left)) p_left = 0; if (!std::isfinite(p_right)) p_right = 0;
    if (p_left < 0) p_left = 0; else if (p_left > 255) p_left = 255;
    if (p_right < 0) p_right = 0; else if (p_right > 255) p_right = 255;
    left_motor_pwm = (int)lround(p_left);
    right_motor_pwm = (int)lround(p_right);
    left_motor_dir = vel_left >= 0 ? 1 : 0;
    right_motor_dir = vel_right >= 0 ? 1 : 0;
    result.left = left_motor_pwm;
    result.right = right_motor_pwm;
    return result;
}

void DiffCar::update_h_bridge(){

    if(left_motor_pwm != 0){
        if (left_motor_dir) {
            digitalWrite(MOTOR_IN1, HIGH);
            digitalWrite(MOTOR_IN2, LOW);
            ledcWrite(MOTOR_EN_A, left_motor_pwm);
        } else {
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, HIGH);
            ledcWrite(MOTOR_EN_A, left_motor_pwm);
        }
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_EN_A, 0);
    }

    if(right_motor_pwm != 0){
        if (right_motor_dir) {
            digitalWrite(MOTOR_IN3, HIGH);
            digitalWrite(MOTOR_IN4, LOW);
            ledcWrite(MOTOR_EN_B, right_motor_pwm);
        } else {
            digitalWrite(MOTOR_IN3, LOW);
            digitalWrite(MOTOR_IN4, HIGH);
            ledcWrite(MOTOR_EN_B, right_motor_pwm);
        }
    }
}

void DiffCar::handler_motor(){
    MotorPwmResult temp_pwm = set_motor_speed_msr(this->left_velocity_target, this->right_velocity_target);
    float temp_pwm_left = temp_pwm.left + temp_pwm.left * left_gain/255;
    float temp_pwm_right = temp_pwm.right + temp_pwm.right * right_gain/255;


    left_pid.Compute();
    set_motor_speed(temp_pwm_left, -1);

    right_pid.Compute();
    set_motor_speed(-1, temp_pwm_right);

    update_h_bridge();
}