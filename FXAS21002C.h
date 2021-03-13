/* FXAS21002C
 * Author: Andres Sabas <s@theinventorhouse.org>
 *
 *Basada en:
 * FXOS8700CQ
 *
 * Author: Matt Warner <mlw2224@rit.edu>
 *
 * The FXAS21002C is an gyroscope I2C sensor which is 3.3V tolerant.
 */

#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#include <Adafruit_Sensor.h>
#include <Arduino.h>  // for byte data type

// register addresses FXAS21002C_H_
#define FXAS21002C_H_STATUS 0x00
#define FXAS21002C_H_DR_STATUS 0x07
#define FXAS21002C_H_F_STATUS 0x08
#define FXAS21002C_H_OUT_X_MSB 0x01
#define FXAS21002C_H_OUT_X_LSB 0x02
#define FXAS21002C_H_OUT_Y_MSB 0x03
#define FXAS21002C_H_OUT_Y_LSB 0x04
#define FXAS21002C_H_OUT_Z_MSB 0x05
#define FXAS21002C_H_OUT_Z_LSB 0x06
#define FXAS21002C_H_F_SETUP 0x09
#define FXAS21002C_H_F_EVENT 0x0A
#define FXAS21002C_H_INT_SRC_FLAG 0x0B
#define FXAS21002C_H_WHO_AM_I 0x0C
#define FXAS21002C_H_CTRL_REG0 0x0D
#define FXAS21002C_H_RT_CFG 0x0E
#define FXAS21002C_H_RT_SRC 0x0F
#define FXAS21002C_H_RT_THS 0x10
#define FXAS21002C_H_RT_COUNT 0x11
#define FXAS21002C_H_TEMP 0x12
#define FXAS21002C_H_CTRL_REG1 0x13
#define FXAS21002C_H_CTRL_REG2 0x14
#define FXAS21002C_H_CTRL_REG3 0x15

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
/** 7-bit address for this sensor */
#define FXAS21002C_ADDRESS (0x20)  // 0100001
/** Device ID for this sensor (used as a sanity check during init) */
#define FXAS21002C_ID (0xD7)  // 1101 0111
/** Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS (0.0078125F)  // Table 35 of datasheet
/** Gyroscope sensitivity at 500dps */
#define GYRO_SENSITIVITY_500DPS (0.015625F)
/** Gyroscope sensitivity at 1000dps */
#define GYRO_SENSITIVITY_1000DPS (0.03125F)
/** Gyroscope sensitivity at 2000dps */
#define GYRO_SENSITIVITY_2000DPS (0.0625F)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
    GYRO_REGISTER_STATUS = 0x00,    /**< 0x00 */
    GYRO_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
    GYRO_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
    GYRO_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
    GYRO_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
    GYRO_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
    GYRO_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
    GYRO_REGISTER_WHO_AM_I =
        0x0C, /**< 0x0C (default value = 0b11010111, read only) */
    GYRO_REGISTER_CTRL_REG0 =
        0x0D, /**< 0x0D (default value = 0b00000000, read/write) */
    GYRO_REGISTER_CTRL_REG1 =
        0x13, /**< 0x13 (default value = 0b00000000, read/write) */
    GYRO_REGISTER_CTRL_REG2 =
        0x14, /**< 0x14 (default value = 0b00000000, read/write) */
} gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Enum to define valid gyroscope range values
*/
typedef enum {
    GYRO_RANGE_250DPS = 250,   /**< 250dps */
    GYRO_RANGE_500DPS = 500,   /**< 500dps */
    GYRO_RANGE_1000DPS = 1000, /**< 1000dps */
    GYRO_RANGE_2000DPS = 2000  /**< 2000dps */
} gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
/*!
    Struct to store a single raw (integer-based) gyroscope vector
*/
typedef struct gyroRawData_s {
    int16_t x; /**< Raw int16_t value for the x axis */
    int16_t y; /**< Raw int16_t value for the y axis */
    int16_t z; /**< Raw int16_t value for the z axis */
} gyroRawData_t;
/*=========================================================================*/

enum GyroLPF { GLPS_HIGH = 0, GLPS_MID = 1, GLPS_LOW = 2 };

enum GyroODR {
    GODR_800HZ = 0,  // 200 Hz
    GODR_400HZ,
    GODR_200HZ,
    GODR_100HZ,
    GODR_50HZ,
    GODR_25HZ,  // 12.5 Hz, etc.
    GODR_12_5HZ
};
// Set initial input parameters
enum GyroFSR { GFS_2000DPS = 0, GFS_1000DPS, GFS_500DPS, GFS_250DPS };

class FXAS21002C : public Adafruit_Sensor {
public:
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } SRAWDATA;

    // Sensor data
    SRAWDATA gyroData;  // RAW acceleration sensor data

    float gRes,
        gBias[3] = {0, 0, 0};  // scale resolutions per LSB for the sensors

    FXAS21002C(byte addr);

    // FXAS21002C functions
    // Initialization & Termination
    void init(void);
    void standby(void);
    void active(void);
    void ready(void);

    // Query sensor data
    void readGyroData(void);
    void readTempData(void);

    // Resolution
    float getGres(void);

    int setLowPassLPF(GyroLPF lpf);
    int setRateODR(GyroODR odr);
    int setRate(int rate);

    int setRangeFSR(GyroFSR fsr);
    int setRange(int range);

    int enableDoubleFSR();
    int disableDoubleFSR();

    // Calibrate
    void calibrate(int samples, int hertz);

    // Reset
    void reset(void);

    bool begin(gyroRange_t rng = GYRO_RANGE_250DPS);
    bool getEvent(sensors_event_t *event);
    void getSensor(sensor_t *sensor);
    void standby(boolean standby);

    /*! Raw gyroscope values from last sensor read */
    gyroRawData_t raw;

private:
    // Register functions
    void writeReg(byte reg, byte value);
    byte readReg(byte reg);
    void readRegs(byte startReg, uint8_t count, byte dest[]);

    int8_t tempData;  // RAW temperature data

    // Sensor configuration
    uint8_t gyroFSR = GFS_250DPS;
    uint8_t gyroODR = GODR_200HZ;

    gyroRange_t _range;

    // Sensor address
    byte address;
};

#endif
