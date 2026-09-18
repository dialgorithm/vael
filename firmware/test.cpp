#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>

// ============================================================
// VAEL - ESP32-S3 BASIC FLIGHT CONTROLLER FIRMWARE
//
// Arduino ESP32 core
//
// Functions:
//   - ELRS / CRSF RC receiver
//   - RC failsafe
//   - Arm/disarm
//   - MPU6050
//   - BMP180
//   - MicroSD logging
//   - Four PWM output channels
//
// IMPORTANT:
// This firmware is NOT a flight-ready stabilization controller.
// Do not connect propellers during initial testing.
// ============================================================


// ============================================================
// PIN CONFIGURATION
// ============================================================

// I2C
#define I2C_SDA             8
#define I2C_SCL             9

// ELRS receiver UART
#define ELRS_RX_PIN         18
#define ELRS_TX_PIN         17

// microSD SPI
#define SD_SCK_PIN          12
#define SD_MISO_PIN         13
#define SD_MOSI_PIN         11
#define SD_CS_PIN           10

// Motor / actuator outputs
#define MOTOR_1_PIN         4
#define MOTOR_2_PIN         5
#define MOTOR_3_PIN         6
#define MOTOR_4_PIN         7


// ============================================================
// RC CONFIGURATION
// ============================================================

#define RC_CHANNELS         16

// Typical CRSF channel indexes
#define CH_ROLL             0
#define CH_PITCH            1
#define CH_THROTTLE         2
#define CH_YAW              3

// Change this to match your ELRS transmitter.
// Example: channel 4 used as ARM switch.
#define CH_ARM              4

// CRSF nominal channel range
#define RC_MIN              172
#define RC_MID              992
#define RC_MAX              1811

// Receiver considered lost after this interval
#define RC_TIMEOUT_MS       250


// ============================================================
// PWM CONFIGURATION
// ============================================================

#define PWM_FREQUENCY       400
#define PWM_RESOLUTION      12

#define PWM_MIN             1000
#define PWM_MAX             2000

#define MOTOR_IDLE          1000


// ============================================================
// GLOBAL OBJECTS
// ============================================================

HardwareSerial ELRS(1);

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

bool mpuOK = false;
bool bmpOK = false;
bool sdOK  = false;


// ============================================================
// RC STATE
// ============================================================

uint16_t rcChannels[RC_CHANNELS];

bool receiverConnected = false;
bool armed = false;

uint32_t lastRCFrame = 0;


// ============================================================
// CRSF
// ============================================================

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS     0x16

uint8_t crsfBuffer[64];
uint8_t crsfIndex = 0;
uint8_t crsfLength = 0;


// ============================================================
// LOGGING
// ============================================================

File logFile;

uint32_t lastLogTime = 0;
uint32_t logStartTime = 0;


// ============================================================
// SENSOR VALUES
// ============================================================

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float temperature = 0;
float pressure = 0;
float altitude = 0;


// ============================================================
// MOTOR VALUES
// ============================================================

uint16_t motor1 = MOTOR_IDLE;
uint16_t motor2 = MOTOR_IDLE;
uint16_t motor3 = MOTOR_IDLE;
uint16_t motor4 = MOTOR_IDLE;


// ============================================================
// CRSF CRC8
// ============================================================

uint8_t crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc ^= *ptr++;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }

    return crc;
}


// ============================================================
// CRSF CHANNEL DECODER
// ============================================================

void decodeCRSFChannels(uint8_t *payload)
{
    // CRSF packs 16 channels as 11-bit values.

    uint32_t bits = 0;
    uint8_t bitCount = 0;

    uint8_t channel = 0;

    for (uint8_t i = 0; i < 22 && channel < RC_CHANNELS; i++)
    {
        bits |= ((uint32_t)payload[i]) << bitCount;
        bitCount += 8;

        while (bitCount >= 11 && channel < RC_CHANNELS)
        {
            rcChannels[channel] = bits & 0x07FF;

            bits >>= 11;
            bitCount -= 11;

            channel++;
        }
    }

    receiverConnected = true;
    lastRCFrame = millis();
}


// ============================================================
// CRSF PARSER
// ============================================================

void processCRSF()
{
    while (ELRS.available())
    {
        uint8_t b = ELRS.read();

        // First byte = device address
        if (crsfIndex == 0)
        {
            if (b != CRSF_ADDRESS_FLIGHT_CONTROLLER)
                continue;

            crsfBuffer[crsfIndex++] = b;
            continue;
        }

        // Second byte = frame length
        if (crsfIndex == 1)
        {
            crsfLength = b;

            if (crsfLength < 2 || crsfLength > 62)
            {
                crsfIndex = 0;
                continue;
            }

            crsfBuffer[crsfIndex++] = b;
            continue;
        }

        crsfBuffer[crsfIndex++] = b;

        // frame = address + length + payload + CRC
        if (crsfIndex >= crsfLength + 2)
        {
            uint8_t type = crsfBuffer[2];

            uint8_t crcReceived =
                crsfBuffer[crsfIndex - 1];

            uint8_t crcCalculated =
                crc8(&crsfBuffer[2], crsfLength - 1);

            if (crcReceived == crcCalculated)
            {
                if (type == CRSF_FRAMETYPE_RC_CHANNELS)
                {
                    // RC channels occupy bytes 3-24
                    decodeCRSFChannels(&crsfBuffer[3]);
                }
            }

            crsfIndex = 0;
        }
    }
}


// ============================================================
// RC HELPERS
// ============================================================

float rcToNormalized(uint16_t value)
{
    float x = ((float)value - RC_MID) /
              ((float)RC_MAX - RC_MIN) * 2.0f;

    return constrain(x, -1.0f, 1.0f);
}


float rcThrottle()
{
    float t =
        ((float)rcChannels[CH_THROTTLE] - RC_MIN) /
        ((float)RC_MAX - RC_MIN);

    return constrain(t, 0.0f, 1.0f);
}


bool armSwitchActive()
{
    return rcChannels[CH_ARM] > 1500;
}


// ============================================================
// FAILSAFE
// ============================================================

bool rcFailsafe()
{
    if (!receiverConnected)
        return true;

    if (millis() - lastRCFrame > RC_TIMEOUT_MS)
        return true;

    return false;
}


// ============================================================
// ARMING
// ============================================================

void updateArming()
{
    if (rcFailsafe())
    {
        armed = false;
        return;
    }

    // Require low throttle before arming.
    bool throttleLow = rcThrottle() < 0.05f;

    if (!armed)
    {
        if (armSwitchActive() && throttleLow)
        {
            armed = true;
        }
    }
    else
    {
        if (!armSwitchActive())
        {
            armed = false;
        }
    }
}


// ============================================================
// MOTOR OUTPUT
// ============================================================
//
// This function deliberately does NOT implement a flight
// controller. It provides a controlled output interface.
//
// Until stabilization/mixer code is added, outputs remain at
// minimum when disarmed.
//
// This prevents raw transmitter commands from directly
// driving motors during initial hardware testing.
//

void writeMotor(uint8_t pin, uint16_t us)
{
    us = constrain(us, PWM_MIN, PWM_MAX);

    // 400 Hz PWM:
    //
    // period = 2500 us
    //
    // 12-bit duty:
    // duty = pulse / 2500 * 4095

    uint32_t duty =
        ((uint32_t)us * 4095UL) / 2500UL;

    ledcWrite(pin, duty);
}


void updateMotors()
{
    if (!armed || rcFailsafe())
    {
        motor1 = MOTOR_IDLE;
        motor2 = MOTOR_IDLE;
        motor3 = MOTOR_IDLE;
        motor4 = MOTOR_IDLE;
    }
    else
    {
        // ----------------------------------------------------
        // BENCH OUTPUT MODE
        //
        // Throttle is mapped to all four outputs equally.
        //
        // This is NOT a quadcopter mixer or stabilization loop.
        // ----------------------------------------------------

        uint16_t throttle =
            PWM_MIN +
            rcThrottle() * (PWM_MAX - PWM_MIN);

        motor1 = throttle;
        motor2 = throttle;
        motor3 = throttle;
        motor4 = throttle;
    }

    writeMotor(MOTOR_1_PIN, motor1);
    writeMotor(MOTOR_2_PIN, motor2);
    writeMotor(MOTOR_3_PIN, motor3);
    writeMotor(MOTOR_4_PIN, motor4);
}


// ============================================================
// MPU6050
// ============================================================

void readMPU6050()
{
    if (!mpuOK)
        return;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);

    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;

    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;

    temperature = temp.temperature;
}


// ============================================================
// BMP180
// ============================================================

void readBMP180()
{
    if (!bmpOK)
        return;

    pressure = bmp.readPressure();

    altitude =
        bmp.readAltitude(101325.0);
}


// ============================================================
// SD LOGGING
// ============================================================

void startLogging()
{
    if (!sdOK)
        return;

    String filename = "/flight.csv";

    logFile = SD.open(filename, FILE_APPEND);

    if (!logFile)
        return;

    // Write header if file is empty.
    if (logFile.size() == 0)
    {
        logFile.println(
            "time_ms,"
            "armed,"
            "failsafe,"
            "ch1,"
            "ch2,"
            "ch3,"
            "ch4,"
            "accel_x,"
            "accel_y,"
            "accel_z,"
            "gyro_x,"
            "gyro_y,"
            "gyro_z,"
            "pressure,"
            "altitude,"
            "motor1,"
            "motor2,"
            "motor3,"
            "motor4"
        );

        logFile.flush();
    }

    logStartTime = millis();
}


void logData()
{
    if (!sdOK || !logFile)
        return;

    if (millis() - lastLogTime < 20)
        return;

    lastLogTime = millis();

    logFile.print(millis());
    logFile.print(",");

    logFile.print(armed);
    logFile.print(",");

    logFile.print(rcFailsafe());
    logFile.print(",");

    for (int i = 0; i < 4; i++)
    {
        logFile.print(rcChannels[i]);
        logFile.print(",");
    }

    logFile.print(accelX, 4);
    logFile.print(",");

    logFile.print(accelY, 4);
    logFile.print(",");

    logFile.print(accelZ, 4);
    logFile.print(",");

    logFile.print(gyroX, 4);
    logFile.print(",");

    logFile.print(gyroY, 4);
    logFile.print(",");

    logFile.print(gyroZ, 4);
    logFile.print(",");

    logFile.print(pressure, 2);
    logFile.print(",");

    logFile.print(altitude, 2);
    logFile.print(",");

    logFile.print(motor1);
    logFile.print(",");

    logFile.print(motor2);
    logFile.print(",");

    logFile.print(motor3);
    logFile.print(",");

    logFile.println(motor4);

    // Don't flush every line.
    // Flush periodically for better SD performance.
    static uint32_t lastFlush = 0;

    if (millis() - lastFlush > 1000)
    {
        logFile.flush();
        lastFlush = millis();
    }
}


// ============================================================
// SERIAL DEBUG
// ============================================================

void printStatus()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint < 500)
        return;

    lastPrint = millis();

    Serial.print("RC: ");

    for (int i = 0; i < 6; i++)
    {
        Serial.print(rcChannels[i]);
        Serial.print(" ");
    }

    Serial.print("| ARM=");
    Serial.print(armed);

    Serial.print(" FS=");
    Serial.print(rcFailsafe());

    Serial.print(" | ACC=");
    Serial.print(accelX, 2);
    Serial.print(",");
    Serial.print(accelY, 2);
    Serial.print(",");
    Serial.print(accelZ, 2);

    Serial.print(" | ALT=");
    Serial.print(altitude, 2);

    Serial.print(" | MOT=");
    Serial.print(motor1);
    Serial.print(",");
    Serial.print(motor2);
    Serial.print(",");
    Serial.print(motor3);
    Serial.print(",");
    Serial.println(motor4);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("VAEL FLIGHT CONTROLLER");
    Serial.println("ESP32-S3");
    Serial.println("================================");

    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(I2C_SDA, I2C_SCL);

    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    if (mpu.begin(0x68, &Wire))
    {
        mpuOK = true;

        mpu.setAccelerometerRange(
            MPU6050_RANGE_8_G
        );

        mpu.setGyroRange(
            MPU6050_RANGE_500_DEG
        );

        mpu.setFilterBandwidth(
            MPU6050_BAND_44_HZ
        );

        Serial.println("MPU6050: OK");
    }
    else
    {
        Serial.println("MPU6050: FAIL");
    }

    // --------------------------------------------------------
    // BMP180
    // --------------------------------------------------------

    if (bmp.begin())
    {
        bmpOK = true;
        Serial.println("BMP180: OK");
    }
    else
    {
        Serial.println("BMP180: FAIL");
    }

    // --------------------------------------------------------
    // ELRS / CRSF
    // --------------------------------------------------------

    //
    // CRSF is normally:
    //
    // ELRS TX -> ESP32 RX
    // ELRS RX -> ESP32 TX
    // GND     -> GND
    //
    // Many ELRS receivers only require TX -> FC RX
    // for RC input.
    //

    ELRS.begin(
        420000,
        SERIAL_8N1,
        ELRS_RX_PIN,
        ELRS_TX_PIN
    );

    Serial.println("ELRS UART: started");

    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledcAttach(
        MOTOR_1_PIN,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR_2_PIN,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR_3_PIN,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttach(
        MOTOR_4_PIN,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    // Always start outputs at minimum.
    writeMotor(MOTOR_1_PIN, MOTOR_IDLE);
    writeMotor(MOTOR_2_PIN, MOTOR_IDLE);
    writeMotor(MOTOR_3_PIN, MOTOR_IDLE);
    writeMotor(MOTOR_4_PIN, MOTOR_IDLE);

    // --------------------------------------------------------
    // SD
    // --------------------------------------------------------

    SPI.begin(
        SD_SCK_PIN,
        SD_MISO_PIN,
        SD_MOSI_PIN,
        SD_CS_PIN
    );

    if (SD.begin(SD_CS_PIN, SPI))
    {
        sdOK = true;
        Serial.println("microSD: OK");

        startLogging();
    }
    else
    {
        Serial.println("microSD: FAIL");
    }

    // --------------------------------------------------------
    // Initialize RC
    // --------------------------------------------------------

    for (int i = 0; i < RC_CHANNELS; i++)
        rcChannels[i] = RC_MID;

    rcChannels[CH_THROTTLE] = RC_MIN;

    Serial.println("VAEL initialization complete.");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // Highest priority: receive RC data.
    processCRSF();

    // Read sensors.
    readMPU6050();
    readBMP180();

    // Update arm/failsafe state.
    updateArming();

    // Update outputs.
    updateMotors();

    // Record flight data.
    logData();

    // USB serial diagnostics.
    printStatus();

    delay(1);
}