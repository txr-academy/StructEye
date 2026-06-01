#include "mpu6050.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // Check device ID
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) {
        // Power management 1: wake up device
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +/- 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +/- 250 deg/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        // Disable Data Ready Interrupt (we poll, not interrupt-driven)
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);

        return 1; // Success
    }
    return 0; // Failed
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14] = {0};
    
    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 100) != HAL_OK) {
        // I2C Read failed! Return zeros so we don't process random stack garbage
        DataStruct->Accel_X = 0;
        DataStruct->Accel_Y = 0;
        DataStruct->Accel_Z = 0;
        DataStruct->Gyro_X = 0;
        DataStruct->Gyro_Y = 0;
        DataStruct->Gyro_Z = 0;
        return;
    }

    DataStruct->Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    int16_t temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Temperature = (float)temp / 340.0 + 36.53;
    
    DataStruct->Gyro_X = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
}
