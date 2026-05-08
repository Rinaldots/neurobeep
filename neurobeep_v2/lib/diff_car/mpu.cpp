#include "diff_car.h"
// Converter giroscópio de deg/s para rad/s
HardwareSerial mpuSerial(2); // Create an object for UART2
void DiffCar::setup_mpu() {

    Wire.begin(MPU_9250_SDA, MPU_9250_SCL, 100000); // 100kHz para melhor estabilidade
    delay(1000); 
    Serial.println("Tentando conectar ao MPU9250...");
    Wire.beginTransmission(0x68);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("MPU9250 detectado no endereço 0x68");
    } else {
        Serial.print("Erro na comunicação I2C: ");
        Serial.println(error);
        Wire.beginTransmission(0x69);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.println("MPU9250 detectado no endereço 0x69");
        } else {
            Serial.println("MPU9250 não encontrado em nenhum endereço");
            return; 
        }
    }
    
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_184HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_218HZ_0;

     if (!mpu.setup(error == 0 ? 0x68 : 0x69, setting)) {
        Serial.println("Falha na configuração do MPU. Continuando sem MPU...");
        // Em vez de travar, apenas registrar o erro e continuar
        this->mpu_available = false;
    } else {
        Serial.println("MPU connection success.");
        this->mpu_available = true;
    }
}
void DiffCar::correct_imu()
{
    float accel_x_raw = mpu.getAccX();
    float accel_y_raw = mpu.getAccY();
    float accel_z_raw = mpu.getAccZ();

    float gyro_x_raw = mpu.getGyroX();
    float gyro_y_raw = mpu.getGyroY();
    float gyro_z_raw = mpu.getGyroZ();

    float mag_x_raw = mpu.getMagX();  
    float mag_y_raw = mpu.getMagY();
    float mag_z_raw = mpu.getMagZ();
        
    
    gyro_x_raw *= DEG_TO_RAD;
    gyro_y_raw *= DEG_TO_RAD;
    gyro_z_raw *= DEG_TO_RAD;
    
    // Aplicar transformação de rotação para aceleração
    this->mpu_accel.linear.x = accel_y_raw;  // x_odom = y_imu
    this->mpu_accel.linear.y = accel_x_raw;  // y_odom = x_imu
    this->mpu_accel.linear.z = accel_z_raw;  // z_odom = z_imu
    
    // Aplicar transformação de rotação para velocidade angular (já em rad/s)
    this->mpu_accel.angular.x = gyro_y_raw;  // x_odom = y_imu
    this->mpu_accel.angular.y = gyro_x_raw;  // y_odom = x_imu  
    this->mpu_accel.angular.z = gyro_z_raw;  // z_odom = z_imu
    
    this->mpu_accel.magnetic.x = mag_y_raw;  // x_odom = y_imu
    this->mpu_accel.magnetic.y = mag_x_raw;  // y_odom = x_imu  
    this->mpu_accel.magnetic.z = mag_z_raw;  // z_odom = z_imu
}

void DiffCar::update_mpu() {
    if (!this->mpu_available) return;
    if (mpu.update()) {
        this->correct_imu();
    }
}
void DiffCar::calibrate_imu() {
    if (!this->mpu_available) return;
    
    Serial.println("Starting Advanced IMU Calibration...");
    Serial.println("Keep the device STATIONARY and LEVEL during calibration!");
    delay(2000);
    
    // Calibração do Giroscópio (5 loops)
    Serial.println("Calibrating Gyroscope...");
    this->calibrate_gyro(10);
    
    // Calibração do Acelerômetro (5 loops)  
    Serial.println("Calibrating Accelerometer...");
    this->calibrate_accel(5);
    
    Serial.println("Calculating and setting Magnetometer Offsets...");
    this->calibrate_magnetometer(2);

    Serial.println("Advanced IMU Calibration complete!");


}

void DiffCar::debug_mpu() {
    Serial.print("Accel [g]: ");
    Serial.print(this->mpu_accel.linear.x, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.linear.y, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.linear.z, 3);
    Serial.print(" | Gyro [rad/s]: ");  // Atualizado para rad/s
    Serial.print(this->mpu_accel.angular.x, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.angular.y, 3);
    Serial.print(", ");
    Serial.println(this->mpu_accel.angular.z, 3);
}

//***************************************************************************************
//**********************           Calibration Routines            **********************
//***************************************************************************************

/**
 * @brief Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
 */
void DiffCar::calibrate_gyro(uint8_t loops) {
    float kP = 0.3;
    float kI = 90;
    float x = (100 - map(loops, 1, 5, 20, 0)) * 0.01;
    kP *= x;
    kI *= x;
    
    Serial.print("Gyro calibration: ");
    this->pid_calibration(0x43, kP, kI, loops);  // 0x43 = GYRO_XOUT_H register
    Serial.println(" Done!");
}

/**
 * @brief Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
 */
void DiffCar::calibrate_accel(uint8_t loops) {
    float kP = 0.3;
    float kI = 20;
    float x = (100 - map(loops, 1, 5, 20, 0)) * 0.01;
    kP *= x;
    kI *= x;
    
    Serial.print("Accel calibration: ");
    this->pid_calibration(0x3B, kP, kI, loops);  // 0x3B = ACCEL_XOUT_H register
    Serial.println(" Done!");
}
void DiffCar::calibrate_magnetometer(uint8_t loops) {
    float kP = 0.3;
    float kI = 20;
    float x = (100 - map(loops, 1, 5, 20, 0)) * 0.01;
    kP *= x;
    kI *= x;

    Serial.print("Magnetometer calibration: ");
    this->pid_calibration(0x48, kP, kI, loops);  // 0x3B = ACCEL_XOUT_H register
    Serial.println(" Done!");
}
/**
 * @brief PID-based calibration algorithm for MPU6050/MPU9250
 */
void DiffCar::pid_calibration(uint8_t readAddress, float kP, float kI, uint8_t loops) {
    uint8_t saveAddress = (readAddress == 0x3B) ? 0x77 : 0x13;  // Accel offset or Gyro offset registers
    int16_t data;
    float reading;
    int16_t bitZero[3];
    uint8_t shift = (saveAddress == 0x77) ? 3 : 2;
    float error, pTerm, iTerm[3];
    int16_t eSample;
    uint32_t eSum;
    
    uint8_t mpuAddress = 0x68;  // Default MPU address
    
    Serial.write('*');
    
    // Initialize offset values
    for (int i = 0; i < 3; i++) {
        // Read current offset value
        Wire.beginTransmission(mpuAddress);
        Wire.write(saveAddress + (i * shift));
        Wire.endTransmission(false);
        Wire.requestFrom(mpuAddress, 2);
        
        if (Wire.available() >= 2) {
            data = (Wire.read() << 8) | Wire.read();
            reading = data;
            
            if (saveAddress != 0x13) {  // Accelerometer
                bitZero[i] = data & 1;  // Capture Bit Zero for proper accel calibration
                iTerm[i] = ((float)data) * 8;
            } else {  // Gyroscope
                iTerm[i] = ((float)data) * 4;
            }
        }
    }
    
    // Main calibration loops
    for (int L = 0; L < loops; L++) {
        eSample = 0;
        for (int c = 0; c < 100; c++) {  // 100 PI Calculations
            eSum = 0;
            for (int i = 0; i < 3; i++) {
                // Read sensor data
                Wire.beginTransmission(mpuAddress);
                Wire.write(readAddress + (i * 2));
                Wire.endTransmission(false);
                Wire.requestFrom(mpuAddress, 2);
                if (Wire.available() >= 2) {
                    data = (Wire.read() << 8) | Wire.read();
                    reading = data;
                    if ((readAddress == 0x3B) && (i == 2)) reading -= 16384;  // Remove gravity from Z-axis accel
                    error = -reading;  // PID is reverse
                    eSum += (reading < 0) ? error : reading;  // Only positive numbers
                    pTerm = kP * error;
                    iTerm[i] += (error * 0.001) * kI;  // Integral term
                    if (saveAddress != 0x13) {  // Accelerometer
                        data = round((pTerm + iTerm[i]) / 8);
                        if (abs(error) > 400) {
                            if ((L + c) == 0) {
                                data = 0;
                                iTerm[i] = data;
                            } else if ((L == 0) && (c == 1)) {
                                data = error / 8;
                                iTerm[i] = data;
                            }
                        }
                        data = ((data) & 0xFFFE) | bitZero[i];  // Insert saved Bit0
                    } else {  // Gyroscope
                        data = round((pTerm + iTerm[i]) / 4);
                    }
                    // Write offset value
                    Wire.beginTransmission(mpuAddress);
                    Wire.write(saveAddress + (i * shift));
                    Wire.write((data >> 8) & 0xFF);
                    Wire.write(data & 0xFF);
                    Wire.endTransmission();
                }
            }
            if ((c == 99) && eSum > 1000) {
                c = 2;
                Serial.write('-');
            }
            if ((eSum * ((readAddress == 0x3B) ? 0.05 : 1)) < 5) eSample++;
            if ((eSum < 100) && (c > 10) && (eSample >= 10)) break;
            delay(1);
        }
        Serial.write('.');
        kP *= 0.75;
        kI *= 0.75;
    }
    
    // Final offset calculation
    for (int i = 0; i < 3; i++) {
        if (saveAddress != 0x13) {  // Accelerometer
            data = round((iTerm[i]) / 8);
            data = ((data) & 0xFFFE) | bitZero[i];
        } else {  // Gyroscope
            data = round((iTerm[i]) / 4);
        }
        
        // Write final offset
        Wire.beginTransmission(mpuAddress);
        Wire.write(saveAddress + (i * shift));
        Wire.write((data >> 8) & 0xFF);
        Wire.write(data & 0xFF);
        Wire.endTransmission();
    }
}


