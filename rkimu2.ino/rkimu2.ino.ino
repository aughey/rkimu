#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <EEPROM.h>

WiFiUDP Udp;
OSCBundle oscbundle;
unsigned long nextloop;
int button_down = 0;

int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {1,1,1};
int32_t eeprom_key = 31415;

#define LOOP_HZ 20
#define LSM9DS0_REGISTER_CTRL_REG5_G          0x24
#define LSM9DS0_REGISTER_FIFO_CTRL_REG        0x2E
#define LSM9DS0_REGISTER_SRC_REG              0x2F
#define LSM9DS0_REGISTER_CTRL_REG0_XM         0x1F

/* Assign a unique base ID for this sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);

  {
    int offset = 0;
    int key;
    // read the bias and scale from EEPROM
    EEPROM.begin(512);
    EEPROM.get(offset, key); // KEY
    Serial.print("EEPROM Key is: "); Serial.print(key);  Serial.print(" expecting: "); Serial.println(eeprom_key);
    if (key == eeprom_key) {
      int ii;
      Serial.println("Got valid key, reading magnetic bias and scale");
      offset += sizeof(int);
      for (ii = 0; ii < 3; ii++) {
        int32_t v;
        EEPROM.get(offset, v);
        mag_bias[ii] = v;
        offset += sizeof(v);
      }
      for (ii = 0; ii < 3; ii++) {
        int32_t v;
        EEPROM.get(offset, v);
        mag_scale[ii] = v;
        offset += sizeof(v);
      }
    } else {
      Serial.println("magnetic bias not set in EEPROM\n");
    }
  }

  if (false) {
    pinMode(LED_BUILTIN, OUTPUT);
    WiFi.begin("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", "XXXXXXXXXXX");
    int led = HIGH;
    digitalWrite(LED_BUILTIN, led);
    while (WiFi.status() != WL_CONNECTED) {
      led = led == HIGH ? LOW : HIGH;
      digitalWrite(LED_BUILTIN, led);
      digitalWrite(2, led);
      delay(100);

    }
    Serial.write("WIFI CONNECTED ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_BUILTIN, LOW); // ON
  } else {
     pinMode(LED_BUILTIN, OUTPUT);
     WiFi.softAP("RKSensor","johnaughey");
     Serial.println(WiFi.softAPIP());

    digitalWrite(LED_BUILTIN, LOW); // ON
  }
  Udp.begin(4567);

  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  /* Display some basic information on this sensor */
 // displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  Serial.println("Setting up FIFO modes");
  // Turn on STREAM mode for the gyro fifo
  lsm.write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG5_G, 0x40); // Enable FIFO
  lsm.write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG0_XM, 0x40); // Enable FIFO accel

  lsm.write8(GYROTYPE, LSM9DS0_REGISTER_FIFO_CTRL_REG, 0x4f); // Setup STreaming gyro
  lsm.write8(XMTYPE, LSM9DS0_REGISTER_FIFO_CTRL_REG, 0x4f); // Setup STreaming accel

  //lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F); // 95Hz
  lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, 0x4F); // 190Hz
  //lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, 0x8F); // 380Hz


  /* We're ready to go! */
  Serial.println("");

  nextloop = millis() + 1000 / LOOP_HZ; // 100Hz send clock
}

int gyroNumReady() {
  byte reg = lsm.read8(GYROTYPE, LSM9DS0_REGISTER_SRC_REG);
  // WTM OVRN EMPTY FSS4 || FSS3 FSS2 FSS1 FSS0

  // Check for overrun
  if ((reg & 0x40)) {
    Serial.println("OVERRUN");
  }
  return reg & 0x1f;
}

int accelNumReady() {
  byte reg = lsm.read8(XMTYPE, LSM9DS0_REGISTER_SRC_REG);
  // WTM OVRN EMPTY FSS4 || FSS3 FSS2 FSS1 FSS0

  // Check for overrun
  if ((reg & 0x40)) {
    Serial.println("OVERRUN");
  }
  return reg & 0x1f;
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
unsigned int count = 0;
unsigned int firstread = 0;
int nodatacount = 0;

int readGyro() {
  int count = gyroNumReady();
  for (int i = 0; i < count; ++i) {
    lsm.readGyro();
    sensors_event_t gyro;
    lsm.getGyroEvent(&gyro, 0);
    oscbundle.add("/gyro").add(gyro.gyro.x).add(gyro.gyro.y).add(gyro.gyro.z);
  }
  return count;
}

int readAccel() {
  int count = accelNumReady();
  for (int i = 0; i < count; ++i) {
    lsm.readAccel();
    sensors_event_t accel;
    lsm.getAccelEvent(&accel, 0);
    oscbundle.add("/accel").add(accel.acceleration.x).add(accel.acceleration.y).add(accel.acceleration.z);
  }
  return count;
}

int readMag() {
  int16_t mag[3];
  float out[3];
  readRawMagData(mag);
  for(int i=0;i<3;++i) {
    out[i] = ((float)mag[i] - (float)mag_bias[i]) / (float)mag_scale[i];
  }

  oscbundle.add("/mag").add(out[0]).add(out[1]).add(out[2]);
  return 1;
}

void readRawMagData(int16_t *data) {
  lsm.readMag();
  data[0] = lsm.magData.x;
  data[1] = lsm.magData.y;
  data[2] = lsm.magData.z;
}

void printV16(char *prefix, int16_t *values) {
  Serial.print(prefix);
  Serial.print(values[0]); Serial.print(" ");
  Serial.print(values[1]); Serial.print(" ");
  Serial.print(values[2]); Serial.print("\n");
}

void printV32(char *prefix, int32_t *values) {
  Serial.print(prefix);
  Serial.print(values[0]); Serial.print(" ");
  Serial.print(values[1]); Serial.print(" ");
  Serial.print(values[2]); Serial.print("\n");
}

void calibrateMag() {
  // Adapted from https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

  uint16_t ii = 0, sample_count = 0;
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  int32_t dest1[3];
  int32_t dest2[3];

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  sample_count = 50 /*hz*/ * 15;
  for (ii = 0; ii < sample_count; ii++) {
    readRawMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(1000 / 50); // 50hz read on mag
  }


  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = mag_bias[0]; // * MPU9250mRes * MPU9250magCalibration[0]; // save mag biases in G for main program
  dest1[1] = mag_bias[1]; // * MPU9250mRes * MPU9250magCalibration[1];
  dest1[2] = mag_bias[2]; // * MPU9250mRes * MPU9250magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = (/*avg_rad / ((float) */ mag_scale[0]);
  dest2[1] = (/*avg_rad / ((float) */ mag_scale[1]);
  dest2[2] = (/*avg_rad / ((float) */mag_scale[2]);

  printV16("mag_min: ",mag_min);
  printV16("mag_max: ",mag_max);
  printV32("Bias: ",mag_bias);
  printV32("Scale: ",mag_scale);

  int offset = 0;
  EEPROM.put(offset, eeprom_key); // KEY
  offset += sizeof(eeprom_key);
  for (ii = 0; ii < 3; ii++) {
    Serial.println(offset);
    EEPROM.put(offset, mag_bias[ii]);
    offset += sizeof(mag_bias[0]);
  }
  for (ii = 0; ii < 3; ii++) {
    Serial.println(offset);
    EEPROM.put(offset, mag_scale[ii]);
    offset += sizeof(mag_scale[0]);
  }
  EEPROM.commit();

  Serial.println("Mag Calibration done!");
}


bool received_udp = false;
void loop(void)
{
  readAccel();
  readMag();
  readGyro();

  char packet[16];
  int len = Udp.read(packet, 16);
  if (len > 0) {
    received_udp = true;
    digitalWrite(LED_BUILTIN, HIGH); // OFF
    pinMode(LED_BUILTIN, INPUT);
    Serial.println("Received udp packet");
  }

  if (received_udp) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    oscbundle.send(Udp);
    Udp.endPacket();

    // Check the status of the button
    if (digitalRead(LED_BUILTIN)) {
      // This is the release state, if the button
      // was previously held down for 1 second, calibrate
      // magnetometer
      if (button_down > LOOP_HZ) {
        calibrateMag();
      }
      button_down = 0;
    } else {
      button_down++;
    }
  }


  // Clear out our send bundle
  oscbundle.empty();

  int sleeptime = nextloop - millis();
  if (sleeptime > 0) {
    delay(sleeptime);
  }
  nextloop = nextloop + 1000 / LOOP_HZ;
  return;
  /*
    if (isDataAvailable() == false) {
      nodatacount++;
      //  delay(10);
      return;
    }
    int thisread = millis();
    if (count == 0) {
      firstread = thisread;
    }
    count++;

    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);

    int diff = thisread - firstread;
    if (diff > 3000) {
      // print out accelleration data
      Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
      Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
      Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

      Serial.print("Accel Total: "); Serial.println(
        sqrt(
          accel.acceleration.x * accel.acceleration.x +
          accel.acceleration.y * accel.acceleration.y +
          accel.acceleration.z * accel.acceleration.z));

      // print out magnetometer data
      Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
      Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
      Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");

      // print out gyroscopic data
      Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
      Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
      Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

      // print out temperature data
      Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

      Serial.println("**********************\n");
      Serial.print("Samples per second: ");
      Serial.println((float)count / (diff) * 1000);
      Serial.print("ms per sample: ");
      Serial.println((float)diff / (float)count);
      Serial.print("Data count: "); Serial.println(count);
      Serial.print("No data count: "); Serial.println(nodatacount);

      nodatacount = 0;
      firstread = thisread;
      count = 0;

    }
  */
}
