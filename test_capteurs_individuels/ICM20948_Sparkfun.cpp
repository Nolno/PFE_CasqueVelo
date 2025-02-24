#include "ICM_20948.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C imu;

// Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; // 250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {74.3, 153.8, -5.5};

// Accel scale: divide by 16604.0 to normalize
float A_B[3] = {79.60, -18.56, 383.31};
float A_Ainv[3][3] = {
  {1.00847, 0.00470, -0.00428},
  {0.00470, 1.00846, -0.00328},
  {-0.00428, -0.00328, 0.99559}
};

// Mag scale divide by 369.4 to normalize
float M_B[3] = {-156.70, -52.79, -141.07};
float M_Ainv[3][3] = {
  {1.12823, -0.01142, 0.00980},
  {-0.01142, 1.09539, 0.00927},
  {0.00980, 0.00927, 1.10625}
};

// Local magnetic declination in degrees
float declination = -14.84;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; // micros() timers for AHRS loop
float deltat = 0;  // loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; // Euler angle output

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
void calibrateMag();

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }

  // Calibration du magnétomètre
  calibrateMag();
}

void loop() {
  static int loop_counter = 0; // sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; // centered and scaled gyro/accel/mag data

  // Update the sensor values whenever new data is available
  if (imu.dataReady()) {
    imu.getAGMT();
    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
    Mxyz[1] = -Mxyz[1]; // reflect Y and Z
    Mxyz[2] = -Mxyz[2]; // must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; // seconds since last update
    last = now;

    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {
      // Define Tait-Bryan angles. Strictly valid only for approximately level movement
      roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      // conventional nav, yaw increases CW from North, corrected for local magnetic declination
      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }
}

// Vector math
float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// Function to subtract offsets and apply scale/correction matrices to IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  // Apply accel offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  // Apply mag offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) {
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  // observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; // calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f) {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }

  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

  // update quaternion with integrated contribution
  // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
  gx = gx * (0.5 * deltat); // pre-multiply common factors
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// Fonction pour calibrer le magnétomètre
void calibrateMag() {
  Serial.println("Calibrating magnetometer...");
  int32_t magMin[3] = {32767, 32767, 32767};
  int32_t magMax[3] = {-32768, -32768, -32768};

  for (int i = 0; i < 1000; i++) {
    imu.getAGMT(); // Lire les données du magnétomètre
    int16_t magData[3] = {imu.agmt.mag.axes.x, imu.agmt.mag.axes.y, imu.agmt.mag.axes.z};

    // Mettre à jour les valeurs min et max
    if (magData[0] < magMin[0]) magMin[0] = magData[0];
    if (magData[1] < magMin[1]) magMin[1] = magData[1];
    if (magData[2] < magMin[2]) magMin[2] = magData[2];
    if (magData[0] > magMax[0]) magMax[0] = magData[0];
    if (magData[1] > magMax[1]) magMax[1] = magData[1];
    if (magData[2] > magMax[2]) magMax[2] = magData[2];

    delay(10);
  }

  // Calculer les offsets (hard iron)
  M_B[0] = (magMax[0] + magMin[0]) / 2;
  M_B[1] = (magMax[1] + magMin[1]) / 2;
  M_B[2] = (magMax[2] + magMin[2]) / 2;

  // Calculer la matrice de correction (soft iron)
  float avg_delta_range = ((magMax[0] - magMin[0]) + (magMax[1] - magMin[1]) + (magMax[2] - magMin[2])) / 3.0;
  M_Ainv[0][0] = avg_delta_range / (magMax[0] - magMin[0]);
  M_Ainv[1][1] = avg_delta_range / (magMax[1] - magMin[1]);
  M_Ainv[2][2] = avg_delta_range / (magMax[2] - magMin[2]);

  Serial.print("Mag offsets: ");
  Serial.print(M_B[0]);
  Serial.print(", ");
  Serial.print(M_B[1]);
  Serial.print(", ");
  Serial.println(M_B[2]);

  Serial.print("Mag scale factors: ");
  Serial.print(M_Ainv[0][0]);
  Serial.print(", ");
  Serial.print(M_Ainv[1][1]);
  Serial.print(", ");
  Serial.println(M_Ainv[2][2]);
}
