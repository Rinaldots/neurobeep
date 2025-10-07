
#include "diff_car.h"

#include <stdio.h>
#include <math.h>

QTRSensors qtr;
MPU9250 mpu;


// --- Constantes do filtro ---
static const double T = 0.1;  // dt nominal
static const double P0 = 1.0;
static const double Q0 = 0.005;   // ruído do processo
static const double R0_v = 0.005; // ruído medição v
static const double R0_w = 0.005; // ruído medição w

// Matrizes de covariância
static const double Q[EKF_N*EKF_N] = {
    Q0, 0,  0,
    0,  Q0, 0,
    0,  0,  Q0
};

static const double R[EKF_M*EKF_M] = {
    R0_v, 0,   0,   0,   0,
    0,    R0_w, 0,   0,   0,
    0,    0,    R0_w, 0,   0,
    0,    0,    0,   0.1, 0,
    0,    0,    0,   0,   0.1
};

DiffCar::DiffCar() {}

// --- Inicialização EKF ---
void DiffCar::init_kf(ekf_t *ekf) {
    const double Pdiag[EKF_N] = { P0, P0, P0 };
    ekf_initialize(ekf, Pdiag);
    ekf->x[0] = 0.0; // velocidade linear x
    ekf->x[1] = 0.0; // velocidade linear y
    ekf->x[2] = 0.0; // velocidade angular z
}

// --- Atualização EKF simplificada ---
void DiffCar::update_kalman_filter() {
    // --- Leituras dos sensores ---
    float v_odom = this->odometry.vel_raw.linear.x;  // m/s
    float v_y_odom = this->odometry.vel_raw.linear.y;  // m/s
    float w_odom = this->odometry.vel_raw.angular.z; // rad/s

    float ac_x_b = this->mpu_accel.linear.x;  // aceleração X
    float ac_y_b = this->mpu_accel.linear.y;  // aceleração Y
    float gz_b = this->mpu_accel.angular.z;      // giroscópio Z

    // Deadzone do giroscópio
    const double GYRO_DEADZONE = 0.01;
    if (fabs(gz_b) < GYRO_DEADZONE) gz_b = 0.0;

    // --- Integração do giroscópio ---
    static unsigned long last_time = 0;
    static double theta_imu = 0.0;
    unsigned long now = millis();
    double dt = (last_time == 0) ? T : (now - last_time) / 1000.0;
    last_time = now;

    theta_imu += gz_b * dt; // ângulo integrado (não usado no estado, só medição)

    // --- Vetor de medição z ---
    double z[EKF_M] = { v_odom, v_y_odom, w_odom, ac_x_b, ac_y_b };

    // --- Modelo do estado fx = x + aceleração*dt ---
    double fx[EKF_N] = { ekf.x[0], ekf.x[1], ekf.x[2] };

    // --- Jacobiana F e H ---
    double F[EKF_N*EKF_N] = { 
        1, 0, 0, 
        0, 1, 0, 
        0, 0, 1 
    };
    double H[EKF_M*EKF_N] = {
        1, 0, 0, // v_x -> z[0]
        0, 1, 0, // v_y -> z[1]
        0, 0, 1, // w -> z[2]
        0, 0, 0, // ac_x -> nenhum estado diretamente
        0, 0, 0  // ac_y -> nenhum estado diretamente
    };

    // --- Predição e atualização ---
    ekf_predict(&ekf, fx, F, Q);
    ekf_update(&ekf, z, fx, H, R);

    this->velocity_x_est = ekf.x[0];
    this->velocity_y_est = ekf.x[1];
    this->angular_velocity_est = ekf.x[2];
    // --- Saída ---
    //printf("v_x_est=%.3f v_y_est=%.3f w_est=%.3f\n", ekf.x[0], ekf.x[1], ekf.x[2]);
}

// --- Reset EKF ---
void DiffCar::reset_kalman_filter() {
    init_kf(&ekf);
    //Serial.println("Filtro Kalman resetado!");
}

void DiffCar::setup() {
  setup_mpu();
  calibrate_imu();
  setup_h_bridge();
  setup_rfid();
  setup_line_sensor();
  setup_encoder();
  setup_gps();
  init_kf(&ekf);
}

// Função para enviar dados de telemetria via Bluetooth
String DiffCar::get_telemetry_data() {
    String data = "";
    
    // Dados dos encoders
    data += "ENC_L:" + String(encoder_left_count) + ",";
    data += "ENC_R:" + String(encoder_right_count) + ",";
    
    // Velocidades atuais
    data += "VEL_L:" + String(left_velocity_ms, 3) + ",";
    data += "VEL_R:" + String(right_velocity_ms, 3) + ",";
    
    // Velocidades alvo
    data += "TGT_L:" + String(left_velocity_target, 3) + ",";
    data += "TGT_R:" + String(right_velocity_target, 3) + ",";
    
    // Ganhos PID
    data += "GAIN_L:" + String(left_gain, 2) + ",";
    data += "GAIN_R:" + String(right_gain, 2) + ",";
    
    // PWM dos motores
    data += "PWM_L:" + String(left_motor_pwm, 1) + ",";
    data += "PWM_R:" + String(right_motor_pwm, 1) + ",";
    
    // Odometria (do filtro de Kalman)
    data += "ODOM_X:" + String(odometry.pose.position.x, 3) + ",";
    data += "ODOM_Y:" + String(odometry.pose.position.y, 3) + ",";
    data += "ODOM_TH:" + String(odometry.pose.orientation.z, 3) + ",";
    data += "ODOM_VX:" + String(odometry.vel.linear.x, 3) + ",";
    data += "ODOM_VY:" + String(odometry.vel.linear.y, 3) + ",";
    data += "ODOM_W:" + String(odometry.vel.angular.z, 3) + ",";
    
    // Estados estimados do Kalman
    data += "EKF_VX:" + String(ekf.x[0], 3) + ",";
    data += "EKF_VY:" + String(ekf.x[1], 3) + ",";
    data += "EKF_W:" + String(ekf.x[2], 3) + ",";
    
    // Sensor de linha
    data += "LINE_POS:" + String(line_position_value) + ",";
    data += "LINE_DIST:" + String(line_distance_mm) + ",";
    
    // IMU
    data += "IMU_AX:" + String(mpu_accel.linear.x, 3) + ",";
    data += "IMU_AY:" + String(mpu_accel.linear.y, 3) + ",";
    data += "IMU_AZ:" + String(mpu_accel.linear.z, 3) + ",";
    data += "IMU_GX:" + String(mpu_accel.angular.x, 3) + ",";
    data += "IMU_GY:" + String(mpu_accel.angular.y, 3) + ",";
    data += "IMU_GZ:" + String(mpu_accel.angular.z, 3) + ",";
    
    // RFID
    data += "RFID:" + rfid_uid + ",";
    
    // GPS
    data += "GPS_LAT:" + String(gps_latitude, 6) + ",";
    data += "GPS_LNG:" + String(gps_longitude, 6) + ",";
    data += "GPS_ALT:" + String(gps_altitude, 1) + ",";
    data += "GPS_SPD:" + String(gps_speed, 2) + ",";
    data += "GPS_VAL:" + String(gps_valid ? 1 : 0);
    
    return data;
}