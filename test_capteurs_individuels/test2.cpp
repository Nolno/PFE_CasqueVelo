// #include <Wire.h>
// #include <LSM6DSLSensor.h>
// #include <LSM303AGR_ACC_Sensor.h>
// #include <LSM303AGR_MAG_Sensor.h>
// #include <Arduino.h>
// #include <cmath>
//
// #define SerialPort Serial
// #define I2C2_SCL    22
// #define I2C2_SDA    21
//
// // Définition du bus I2C et des capteurs
// TwoWire dev_i2c = TwoWire(0);
// LSM6DSLSensor AccGyr(&dev_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH);
// LSM303AGR_MAG_Sensor Mag(&dev_i2c);
//
// float yaw = 0.0;
// unsigned long lastTime = 0;
// float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
//
// // Valeurs min/max du magnétomètre pour calibration
// float minX = -32768, maxX = 32767;
// float minY = -32768, maxY = 32767;
// float minZ = -32768, maxZ = 32767;
//
// void calibrateGyro() {
//   int32_t gx, gy, gz;
//   int32_t gyroscope[3];
//   SerialPort.println("Calibrating gyroscope...");
//   for (int i = 0; i < 100; i++) {
//     AccGyr.Get_G_Axes(gyroscope);
//     gx = gyroscope[0];
//     gy = gyroscope[1];
//     gz = gyroscope[2];
//     gyroXOffset += gx;
//     gyroYOffset += gy;
//     gyroZOffset += gz;
//     delay(10);
//   }
//   gyroXOffset /= 100;
//   gyroYOffset /= 100;
//   gyroZOffset /= 100;
//   SerialPort.println("Gyroscope calibration done.");
// }
//
// void calibrateMagnetometer() {
//   SerialPort.println("Move the sensor in all directions to calibrate magnetometer...");
//   unsigned long startTime = millis();
//   while (millis() - startTime < 5000) { // 5 secondes de calibration
//     int32_t magnetometer[3];
//     //   Mag.GetAxes(magnetometer);
//     Mag.GetAxes(magnetometer);
//     int32_t mx = magnetometer[0];
//     int32_t my = magnetometer[1];
//     int32_t mz = magnetometer[2];
//       if (mx < minX) minX = mx;
//       if (mx > maxX) maxX = mx;
//       if (my < minY) minY = my;
//       if (my > maxY) maxY = my;
//       if (mz < minZ) minZ = mz;
//       if (mz > maxZ) maxZ = mz;
//
//     delay(50);
//   }
//   SerialPort.println("Magnetometer calibration done.");
// }
//
// void setup() {
//   SerialPort.begin(115200);
//   dev_i2c.begin(I2C2_SDA, I2C2_SCL);
//
//   if (AccGyr.begin() != LSM6DSL_STATUS_OK) {
//     SerialPort.println("Failed to initialize LSM6DSLSensor");
//     while (1);
//   }
//   AccGyr.Enable_X();
//   AccGyr.Enable_G();
//   Mag.begin();
//   Mag.Enable();
//
//   calibrateGyro();
//   calibrateMagnetometer();
//   lastTime = millis();
// }
//
// void loop() {
//   int32_t accelerometer[3], gyroscope[3], magnetometer[3];
//
//   if (AccGyr.Get_X_Axes(accelerometer) != LSM6DSL_STATUS_OK || AccGyr.Get_G_Axes(gyroscope) != LSM6DSL_STATUS_OK) {
//     SerialPort.println("Sensor read error");
//     return;
//   }
//
//   // Compensation du biais du gyroscope
//   float gx = (gyroscope[0] - gyroXOffset) / 1000.0;
//   float gy = (gyroscope[1] - gyroYOffset) / 1000.0;
//   float gz = (gyroscope[2] - gyroZOffset) / 1000.0;
//
//   // Convertir les valeurs de l’accéléromètre en g
//   float ax = accelerometer[0] / 1000.0;
//   float ay = accelerometer[1] / 1000.0;
//   float az = accelerometer[2] / 1000.0;
//
//   // Calibration du magnétomètre
//   float mx = (magnetometer[0] - minX) / (maxX - minX) * 2 - 1;
//   float my = (magnetometer[1] - minY) / (maxY - minY) * 2 - 1;
//   float mz = (magnetometer[2] - minZ) / (maxZ - minZ) * 2 - 1;
//
//   // Calcul du roll et pitch
//   float roll1  = atan2(ay, az);
//   float pitch1 = atan2(-ax, sqrt(ay * ay + az * az));
//
//   // Correction des valeurs du magnétomètre
//   float Mx_h = mx * cos(pitch1) + mz * sin(pitch1);
//   float My_h = mx * sin(roll1) * sin(pitch1) + my * cos(roll1) - mz * sin(roll1) * cos(pitch1);
//
//   // Calcul de l'azimut (yaw basé sur le magnétomètre)
//   float azimuth = atan2(My_h, Mx_h) * 180.0 / M_PI;
//   if (azimuth < 0) azimuth += 360;
//
//   // Calcul des angles
//   float roll = roll1 * 180.0 / M_PI;
//   float pitch = pitch1 * 180.0 / M_PI;
//
//   // Intégration du gyroscope pour améliorer le yaw
//   unsigned long currentTime = millis();
//   float deltaTime = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;
//   yaw = 0.90 * (yaw + gz * deltaTime) + 0.02 * azimuth; // Filtrage complémentaire
//
//   // Affichage des résultats
//   SerialPort.print("Roll: "); SerialPort.print(roll);
//   SerialPort.print("°, Pitch: "); SerialPort.print(pitch);
//   SerialPort.print("°, Yaw: "); SerialPort.println(yaw);
//
//   delay(50);
// }
