// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// // #include <Adafruit_Simple_AHRS.h>
//
// Adafruit_MPU6050 mpu;
//
// Adafruit_MPU6050_Accelerometer mpu_accel(&mpu);
// Adafruit_MPU6050_Gyro mpu_gyro(&mpu);
// // Adafruit_Simple_AHRS ahrs(&mpu_accel, &mpu_gyro);
// float roll;
// float pitch;
// float heading;
// float x_angle, y_angle, z_angle;
// float x_deviation, y_deviation, z_deviation;
// float x_total_deviations, y_total_deviations, z_total_deviations;
// unsigned long time_interval;
// boolean calibrating = false;
//
// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("Adafruit MPU6050 test!");
//
//     // Try to initialize!
//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) {
//             delay(10);
//         }
//     }
//     Serial.println("MPU6050 Found!");
//
//     //setupt motion detection
//     mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
//     mpu.setMotionDetectionThreshold(1);
//     mpu.setMotionDetectionDuration(20);
//     mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
//     mpu.setInterruptPinPolarity(true);
//     mpu.setMotionInterrupt(true);
//
//     Serial.println("");
//     delay(100);
//     time_interval = millis();
// }
//
// void loop() {
//
//     sensors_vec_t   orientation;
//     sensors_event_t a, g, temp;
//
//     // if (!calibrating)
//     // {
//     //     delay(10000);
//     //     Serial.println("Calibrating Gyro, don't move MPU6050 for 10 seconds");
//     //     unsigned long t = millis();
//     //
//     //     mpu.getEvent(&a, &g, &temp);
//     //     x_angle = g.gyro.x;
//     //     y_angle = g.gyro.y;
//     //     z_angle = g.gyro.z;
//     //
//     //     while (millis() - t < 10000)
//     //     {
//     //         mpu.getEvent(&a, &g, &temp);
//     //     }
//     //     mpu.getEvent(&a, &g, &temp);
//     //     x_deviation = (g.gyro.x - x_angle)/10000;
//     //     y_deviation = (g.gyro.y - y_angle)/10000;
//     //     z_deviation = (g.gyro.z - z_angle)/10000;
//     //
//     //     x_total_deviations += x_angle;
//     //     y_total_deviations += y_angle;
//     //     z_total_deviations += z_angle;
//     //     Serial.print("X_angle:");Serial.print(x_angle);Serial.print(",");
//     //     Serial.print("Y_angle:");Serial.print(y_angle);Serial.print(",");
//     //     Serial.print("Z_angle:");Serial.print(z_angle);Serial.print(",");
//     //     Serial.println();
//     //     Serial.print("X_deviation:");Serial.print(x_deviation);Serial.print(",");
//     //     Serial.print("Y_deviation:");Serial.print(y_deviation);Serial.print(",");
//     //     Serial.print("Z_deviation:");Serial.print(z_deviation);
//     //     Serial.print("\n");
//     //     Serial.println("Calibration done");
//     //     calibrating = true;
//     //     Serial.println("x_angle,y_angle,z_angle,x_raw_angle,y_raw_angle,z_raw_angle,x_total_deviations,y_total_deviations,z_total_deviations");
//     // }
//
//
//     mpu.getEvent(&a, &g, &temp);
//     x_angle += g.gyro.x;
//     y_angle += g.gyro.y;
//     z_angle += g.gyro.z;
//     roll = orientation.roll;
//     pitch = orientation.pitch;
//     heading = orientation.heading;
//     unsigned long time_var = millis() - time_interval;
//     x_total_deviations += x_deviation * time_var;
//     y_total_deviations += y_deviation * time_var;
//     z_total_deviations += z_deviation * time_var;
//
//     // Serial.print("Roll:");Serial.print(roll); Serial.print(",");
//     // Serial.print("Pitch:");Serial.print(pitch);Serial.print(",");
//     // Serial.print("Heading:");Serial.print(heading);Serial.print(",");
//     // Serial.print("AccelX:");Serial.print(a.acceleration.x);Serial.print(",");
//     // Serial.print("AccelY:");Serial.print(a.acceleration.y);Serial.print(",");
//     // Serial.print("AccelZ:");Serial.print(a.acceleration.z);Serial.print(", ");
//     // Serial.print("GyroX:");Serial.print(g.gyro.x);Serial.print(",");
//     // Serial.print("GyroY:");Serial.print(g.gyro.y);Serial.print(",");
//     // Serial.print("GyroZ:");Serial.print(g.gyro.z);Serial.print(", ");
//
//     time_interval = millis();
//     // Serial.print("X_angle:");
//     // Serial.print(x_angle- x_total_deviations);Serial.print(",");
//     // // Serial.print("Y_angle:");
//     // Serial.print(y_angle - y_total_deviations);Serial.print(",");
//     // // Serial.print("Z_angle:");
//     // Serial.print(z_angle - z_total_deviations);Serial.print(",");
//     // Serial.print(x_angle);Serial.print(",");
//     // Serial.print(y_angle);Serial.print(",");
//     // Serial.print(z_angle);Serial.print(",");
//     // Serial.print(x_total_deviations);Serial.print(",");
//     // Serial.print(y_total_deviations);Serial.print(",");
//     // Serial.print(z_total_deviations);
//     //
//     // Serial.print("\n");
//
//     Serial.print("accX:");Serial.print(a.acceleration.x);Serial.print(",");
//     Serial.print("accY:");Serial.print(a.acceleration.y);Serial.print(",");
//     Serial.print("accZ:");Serial.print(a.acceleration.z);Serial.print(",");
//
//     float degX = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
//     float degY = atan2(a.acceleration.x, a.acceleration.z) * 180 / M_PI;
//     float degZ = atan2(a.acceleration.x, a.acceleration.y) * 180 / M_PI;
//
//     Serial.print("degX:");Serial.print(degX);Serial.print(",");
//     Serial.print("degY:");Serial.print(degY);Serial.print(",");
//     Serial.print("degZ:");Serial.print(degZ);Serial.print(",");
//     Serial.print("\n");
//     delay(100);
// }