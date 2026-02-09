#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// Create a new sensor object
BMI270 imu;
float gyro_bias_z = 0;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69

void gyro_init()
{
    // Initialize the I2C library
    Wire1.begin();
    Wire1.setClock(4e5);

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x68)
    while(imu.beginI2C(i2cAddress, Wire1) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    // The accelerometer and gyroscope can be configured with multiple settings
    // to reduce the measurement noise. Both sensors have the following settings
    // in common:
    // .range       - Measurement range. Lower values give more resolution, but
    //                doesn't affect noise significantly, and limits the max
    //                measurement before saturating the sensor
    // .odr         - Output data rate in Hz. Lower values result in less noise,
    //                but lower sampling rates.
    // .filter_perf - Filter performance mode. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // .bwp         - Filter bandwidth parameter. This has several possible
    //                settings that can reduce noise, but cause signal delay
    // 
    // Both sensors have different possible values for each setting:
    // 
    // Accelerometer values:
    // .range       - 2g to 16g
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 12.5Hz to 1600Hz
    //                  Power mode:       0.78Hz to 400Hz
    // .bwp         - Depends on .filter_perf:
    //                  Performance mode: Normal, OSR2, OSR4, CIC
    //                  Power mode:       Averaging from 1 to 128 samples
    // 
    // Gyroscope values:
    // .range       - 125dps to 2000dps (deg/sec)
    // .ois_range   - 250dps or 2000dps (deg/sec) Only relevant when using OIS,
    //                see datasheet for more info. Defaults to 250dps
    // .odr         - Depends on .filter_perf:
    //                  Performance mode: 25Hz to 3200Hz
    //                  Power mode:       25Hz to 100Hz
    // .bwp         - Normal, OSR2, OSR4, CIC
    // .noise_perf  - Similar to .filter_perf. Performance oprtimized mode
    //                results in less noise, but increased power consumption
    // 
    // Note that not all combinations of values are possible. The performance
    // mode restricts which ODR settings can be used, and the ODR restricts some
    // bandwidth parameters. An error code is returned by setConfig, which can
    // be used to determine whether the selected settings are valid.
    int8_t err = BMI2_OK;

    // Set accelerometer config
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_1000;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    // Check whether the config settings above were valid
    while(err != BMI2_OK)
    {
        // Not valid, determine which config was the problem
        if(err == BMI2_E_ACC_INVALID_CFG)
        {
            Serial.println("Accelerometer config not valid!");
        }
        else if(err == BMI2_E_GYRO_INVALID_CFG)
        {
            Serial.println("Gyroscope config not valid!");
        }
        else if(err == BMI2_E_ACC_GYR_INVALID_CFG)
        {
            Serial.println("Both configs not valid!");
        }
        else
        {
            Serial.print("Unknown error: ");
            Serial.println(err);
        }
        delay(1000);
    }

    Serial.println("Configuration valid! Beginning measurements");
    delay(1000);

    Serial.println("Calibrating Gyro Bias... DO NOT MOVE ROBOT");
    delay(500); // Let sensor settle
    
    // 2. CALCULATE AVERAGE OFFSET
    long double sum = 0;
    int samples = 500;
    for(int i=0; i<samples; i++){
        imu.getSensorData();
        sum += imu.data.gyroZ;
        delay(2);
    }
    gyro_bias_z = (float)(sum / samples);
    
    Serial.print("Bias Found: ");
    Serial.println(gyro_bias_z);
}

float get_gyroZ(){
    imu.getSensorData();
    return imu.data.gyroZ - gyro_bias_z;
}

float get_accelZ(){
    imu.getSensorData();
    return imu.data.accelZ;
}

void print_all()
{
    // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    imu.getSensorData();

    // Print acceleration data
    Serial.print("Acceleration in g's");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.accelX, 4);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.accelY, 4);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.print(imu.data.accelZ, 4);

    Serial.print("\t");

    // Print rotation data
    Serial.print("Rotation in deg/sec");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.gyroX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.gyroY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.println(imu.data.gyroZ, 3);

    // Print 50x per second
    delay(20);
}