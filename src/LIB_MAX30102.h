#ifndef LIB_MAX30102_H
#define LIB_MAX30102_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <utility>

// I2C address of the MAX30102
#define MAX30102_ADDRESS       0x57

// I2C speeds
#define I2C_SPEED_STANDARD     100000
#define I2C_SPEED_FAST         400000

// Register addresses (from MAX30102 datasheet)
#define REG_INTR_STATUS_1      0x00
#define REG_INTR_STATUS_2      0x01
#define REG_INTR_ENABLE_1      0x02
#define REG_INTR_ENABLE_2      0x03
#define REG_FIFO_WR_PTR        0x04
#define REG_FIFO_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR        0x06
#define REG_FIFO_DATA          0x07
#define REG_FIFO_CONFIG        0x08
#define REG_MODE_CONFIG        0x09
#define REG_SPO2_CONFIG        0x0A
#define REG_LED1_PA            0x0C
#define REG_LED2_PA            0x0D
#define REG_TEMP_INT           0x1F
#define REG_TEMP_FRAC          0x20
#define REG_TEMP_CONFIG        0x21
#define REG_REV_ID             0xFE
#define REG_PART_ID            0xFF

class MAX30102 {
public:
    // Constructor with optional TwoWire port
    MAX30102(TwoWire &wirePort = Wire);

    // Initialize sensor (incluye LEDs al máximo)
    bool begin(uint8_t i2cAddress = MAX30102_ADDRESS,
               uint32_t i2cSpeed = I2C_SPEED_STANDARD);

    // Sensor power management
    void shutdown();
    void wakeUp();

    // High-level setup: configure LEDs, sample rate, pulse width, ADC range
    void setup();  // ← Nuevo método para configuración predeterminada

    // Configuration
    void setLEDMode(uint8_t mode);
    void setSamplingRate(uint8_t rate);
    void setPulseWidth(uint8_t width);
    void setADCRange(uint8_t range);
    void setLEDPulseAmplitudeRed(uint8_t amplitude);
    void setLEDPulseAmplitudeIR(uint8_t amplitude);

    // FIFO management
    void clearFIFO();
    bool readFIFO(uint32_t &redLED, uint32_t &irLED);

    /**
     *  Read all pending samples in FIFO and return them as a vector of (red, ir) pairs.
     *  @param outSamples  Vector to be filled with all available samples.
     *  @return true if at least one sample was read, false otherwise.
     */
    bool readAllFIFO(std::vector<std::pair<uint32_t, uint32_t>> &outSamples);

    // Temperature
    void startTemperature();
    bool isTemperatureReady();
    float readTemperature();

    // Device information
    uint8_t getRevisionID();
    uint8_t getPartID();

private:
    TwoWire *_wire;
    uint8_t _i2caddr;

    // Low-level I2C
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

    // FIFO helpers
    uint8_t getWritePtr();
    uint8_t getReadPtr();
    uint8_t getFifoCount();
};

#endif // LIB_MAX30102_H