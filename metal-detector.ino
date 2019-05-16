/*
 MPU9250 Breakout --------- Arduino
 VCC ---------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "MPU9250.h"

#include <SimpleKalmanFilter.h>

#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int led    = 13;  // Set up pin 13 led for toggling

#define SERIAL_DEBUG_PORT 115200
#define I2Cclock 400000
#define I2Cport Wire

// Use either this line or the next to select which I2C address your device is using
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

#define CALIBRATE_MAG false
#define DISPLAY_SELF_TEST false

#define MIN_CALIB_DELAY 500
#define NUM_RUNS 20
#define MIN_SENSE_DELAY 200

MPU9250 imu(MPU9250_ADDRESS, I2Cport, I2Cclock);

// TODO: Move to a separate file without breaking anything
class Vector3D {
  public:
    float x, y, z;
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(float arr[]) : x(arr[0]), y(arr[1]), z(arr[2]) {}
    Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

    void operator = (const Vector3D &other) {
      x = other.x;
      y = other.y;
      z = other.z;
    }

    Vector3D operator + (const Vector3D &other) const {
      return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D operator - (const Vector3D &other) const {
      return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    Vector3D operator * (const float &other) const {
      return Vector3D(x * other, y * other, z * other);
    }

    Vector3D operator / (const float &other) const {
      return Vector3D(x / other, y / other, z / other);
    }

    float magnitude() {
      return sqrt(x * x + y * y + z * z);
    }

    Vector3D squared() {
      return Vector3D(x * x, y * y, z * z);
    }

    Vector3D squareRooted() {
      return Vector3D(sqrt(x), sqrt(y), sqrt(z));
    }

    String toString() {
      return "(" + String(x) + ", " + String(y) + ", " + String(z) + ") == " + String(magnitude());
    }

    bool anamolousMagnitude(const Vector3D mean, const Vector3D dev, const int n) {
      float deltaMag = (*this - mean).magnitude();
      float maxDevMag = (dev * (float) n).magnitude();
      bool isBelowDev = deltaMag < -maxDevMag;
      bool isAboveDev = deltaMag > maxDevMag;
      return isBelowDev || isAboveDev;
    }
};

// TODO: Move to a separate file without breaking anything
class KFVector3D {
  public:
    KFVector3D(Vector3D mean, Vector3D dev) {
      KFVector3D(mean, dev, 0.01);
    }

    KFVector3D(Vector3D mean, Vector3D dev, float processVar) {
      xKF = new SimpleKalmanFilter(mean.x, dev.x, processVar);
      yKF = new SimpleKalmanFilter(mean.y, dev.y, processVar);
      zKF = new SimpleKalmanFilter(mean.z, dev.z, processVar);
    }

    Vector3D updateEstimate(Vector3D input) {
      float estX = xKF->updateEstimate(input.x);
      float estY = yKF->updateEstimate(input.y);
      float estZ = zKF->updateEstimate(input.z);
      return Vector3D(estX, estY, estZ);
    }

  private:
    SimpleKalmanFilter *xKF, *yKF, *zKF;
};

Vector3D accelMean, accelDev;
Vector3D gyroMean, gyroDev;
Vector3D magMean, magDev;

KFVector3D *accelKF, *gyroKF, *magKF;

Vector3D meanVector(const Vector3D vectors[], const int count) {
  Vector3D mean = Vector3D();
  for (int i = 0; i < count; i++)
    mean = mean + vectors[i];
  return mean / (float) count;
}

Vector3D stdVector(const Vector3D vectors[], const int count, const Vector3D mean) {
  Vector3D sumOfSquaresVector = Vector3D();
  for (int i = 0; i < count; i++)
    sumOfSquaresVector = sumOfSquaresVector + (vectors[i] - mean).squared();

  Vector3D varianceVector = sumOfSquaresVector / (float) (count - 1);
  return varianceVector.squareRooted();
}

void getCalibrationVectors(
  const int numSamples, const int sensorDelay,
  Vector3D &_accelMean, Vector3D &_accelDev,
  Vector3D &_gyroMean, Vector3D &_gyroDev,
  Vector3D &_magMean, Vector3D &_magDev
) {
  Vector3D accelCalib[numSamples];
  Vector3D gyroCalib[numSamples];
  Vector3D magCalib[numSamples];

  for (int run = 0; run < numSamples;) {
    imu.delt_t = millis() - imu.count;

    if (imu.delt_t >= sensorDelay) {
      accelCalib[run] = readAccel();
      gyroCalib[run] = readGyro();
      magCalib[run] = readMag();

      imu.count = millis();
      run++;
      Serial.println(run);
    }
  }

  _accelMean = meanVector(accelCalib, numSamples);
  _accelDev = stdVector(accelCalib, numSamples, _accelMean);

  _gyroMean = meanVector(gyroCalib, numSamples);
  _gyroDev = stdVector(gyroCalib, numSamples, _gyroMean);

  _magMean = meanVector(magCalib, numSamples);
  _magDev = stdVector(magCalib, numSamples, _magMean);
}

void setup() {
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(SERIAL_DEBUG_PORT);

  while(!Serial) {};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  const byte WHO_AM_I_MPU = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(WHO_AM_I_MPU, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (WHO_AM_I_MPU == 0x71) { // WHO_AM_I should always be 0x71
    Serial.println(F("MPU9250 is online!"));

    if (DISPLAY_SELF_TEST) {
      // Start by performing self test and reporting values
      imu.MPU9250SelfTest(imu.selfTest);
      Serial.print(F("x-axis self test: acceleration trim within : "));
      Serial.print(imu.selfTest[0],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: acceleration trim within : "));
      Serial.print(imu.selfTest[1],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: acceleration trim within : "));
      Serial.print(imu.selfTest[2],1); Serial.println("% of factory value");
      Serial.print(F("x-axis self test: gyration trim within : "));
      Serial.print(imu.selfTest[3],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: gyration trim within : "));
      Serial.print(imu.selfTest[4],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: gyration trim within : "));
      Serial.print(imu.selfTest[5],1); Serial.println("% of factory value");
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

    imu.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println(F("MPU9250 initialized for active data mode...."));

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    const byte WHO_AM_I_AK = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(WHO_AM_I_AK, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (WHO_AM_I_AK != 0x48) {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    imu.initAK8963(imu.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug) {
      //  Serial.println("Calibration values: ");
      Serial.print(F("X-Axis factory sensitivity adjustment value "));
      Serial.println(imu.factoryMagCalibration[0], 2);
      Serial.print(F("Y-Axis factory sensitivity adjustment value "));
      Serial.println(imu.factoryMagCalibration[1], 2);
      Serial.print(F("Z-Axis factory sensitivity adjustment value "));
      Serial.println(imu.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    imu.getAres();
    imu.getGres();
    imu.getMres();

    if (CALIBRATE_MAG) {
      // The next call delays for 4 seconds, and then records about 15 seconds of
      // data to calculate bias and scale.
      imu.magCalMPU9250(imu.magBias, imu.magScale);
      Serial.println(F("AK8963 mag biases (mG)"));
      Serial.println(imu.magBias[0]);
      Serial.println(imu.magBias[1]);
      Serial.println(imu.magBias[2]);

      Serial.println(F("AK8963 mag scale (mG)"));
      Serial.println(imu.magScale[0]);
      Serial.println(imu.magScale[1]);
      Serial.println(imu.magScale[2]);
      delay(2000); // Add delay to see results before serial spew of data
    }

    if(SerialDebug) {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[2], 2);
    }

  } else { // if (WHO_AM_I_MPU == 0x71)
    Serial.print(F("Could not connect to MPU9250: 0x"));
    Serial.println(WHO_AM_I_MPU, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  Serial.println(F("Started calibration procedure..."));

  getCalibrationVectors(
    NUM_RUNS, MIN_CALIB_DELAY,
    accelMean, accelDev,
    gyroMean, gyroDev,
    magMean, magDev
  );

  Serial.println("Acceleration mean: " + accelMean.toString());
  Serial.println("Acceleration dev: " + accelDev.toString());

  Serial.println("Gyroscope mean: " + gyroMean.toString());
  Serial.println("Gyroscope dev: " + gyroDev.toString());

  Serial.println("Magnetometer mean: " + magMean.toString());
  Serial.println("Magnetometer dev: " + magDev.toString());

  accelKF = new KFVector3D(accelMean, accelDev * 3);
  gyroKF = new KFVector3D(gyroMean, gyroDev * 3);
  magKF = new KFVector3D(magMean, magDev * 3);
}

// If intPin goes high, all data registers have new data
// On interrupt, check if data ready interrupt
bool hasNewData() {
  return imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
}

Vector3D readAccel() {
  if (hasNewData()) {
    imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu.ax = (float) imu.accelCount[0] * imu.aRes; // - imu.accelBias[0];
    imu.ay = (float) imu.accelCount[1] * imu.aRes; // - imu.accelBias[1];
    imu.az = (float) imu.accelCount[2] * imu.aRes; // - imu.accelBias[2];
  }

  return Vector3D(imu.ax, imu.ay, imu.az);
}

Vector3D readGyro() {
  if (hasNewData()) {
    imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu.gx = (float) imu.gyroCount[0] * imu.gRes;
    imu.gy = (float) imu.gyroCount[1] * imu.gRes;
    imu.gz = (float) imu.gyroCount[2] * imu.gRes;
  }

  return Vector3D(imu.gx, imu.gy, imu.gz);
}

Vector3D readMag() {
  if (hasNewData()) {
    imu.readMagData(imu.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    // Get actual magnetometer value, this depends on scale being set
    imu.mx = (float) imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] - imu.magBias[0];
    imu.my = (float) imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] - imu.magBias[1];
    imu.mz = (float) imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] - imu.magBias[2];
  }

  return Vector3D(imu.mx, imu.my, imu.mz);
}

void printSensors(const Vector3D &accel, const Vector3D &gyro, const Vector3D &mag) {
  Serial.print("A = ");
  Serial.print(accel.toString());

  Serial.print("     ");

  Serial.print("G = ");
  Serial.print(gyro.toString());

  Serial.print("     ");

  Serial.print("M = ");
  Serial.println(mag.toString());
}

void loop() {
  Serial.println("test");
  Vector3D accel = readAccel();
  Vector3D gyro = readGyro();
  Vector3D mag = readMag();

  accel = accelKF->updateEstimate(accel);
  gyro = gyroKF->updateEstimate(gyro);
  mag = magKF->updateEstimate(mag);

  imu.delt_t = millis() - imu.count;
  if (imu.delt_t >= MIN_SENSE_DELAY) {
    Vector3D accelDelta = accel - accelMean;
    Vector3D gyroDelta = gyro - gyroMean;
    Vector3D magDelta = mag - magMean;

    printSensors(accelDelta, gyroDelta, magDelta);

    // TODO: Fix this algorithm
    if (gyro.anamolousMagnitude(gyroMean, gyroDev, 3)) {
      if (magDelta.x > 50 && magDelta.z > 50)
        Serial.println("ON");
      else
        Serial.println("OFF");
    } else
      Serial.println("OFF");

    imu.count = millis();
    digitalWrite(led, !digitalRead(led));  // toggle led
  }
}