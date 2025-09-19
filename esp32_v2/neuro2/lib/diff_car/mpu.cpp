#include "diff_car.h"

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
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

     if (!mpu.setup(error == 0 ? 0x68 : 0x69, setting)) {
        Serial.println("Falha na configuração do MPU. Continuando sem MPU...");
        // Em vez de travar, apenas registrar o erro e continuar
        this->mpu_available = false;
    } else {
        Serial.println("MPU connection success.");
        this->mpu_available = true;
    }
}
void DiffCar::update_mpu() {
    if (!this->mpu_available) return;
    if (mpu.update()) {
        this->mpu_accel.linear.x = mpu.getAccX();
        this->mpu_accel.linear.y = mpu.getAccY();
        this->mpu_accel.linear.z = mpu.getAccZ();
        this->mpu_accel.angular.x = mpu.getGyroX();
        this->mpu_accel.angular.y = mpu.getGyroY();
        this->mpu_accel.angular.z = mpu.getGyroZ();
    }
}
void DiffCar::debug_mpu() {
    Serial.print("Accel [g]: ");
    Serial.print(this->mpu_accel.linear.x, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.linear.y, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.linear.z, 3);
    Serial.print(" | Gyro [deg/s]: ");
    Serial.print(this->mpu_accel.angular.x, 3);
    Serial.print(", ");
    Serial.print(this->mpu_accel.angular.y, 3);
    Serial.print(", ");
    Serial.println(this->mpu_accel.angular.z, 3);
}


