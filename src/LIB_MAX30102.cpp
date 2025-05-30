#include "LIB_MAX30102.h"

MAX30102::MAX30102(TwoWire &wirePort) {
    _wire = &wirePort;
    _i2caddr = MAX30102_ADDRESS;
}

bool MAX30102::begin(uint8_t i2cAddress, uint32_t i2cSpeed) {
    _i2caddr = i2cAddress;
    _wire->begin();
    _wire->setClock(i2cSpeed);

    uint8_t partID = getPartID();
    if (partID != 0x15) return false;

    // LEDs al máximo para asegurar buena señal AC con dedo
    writeRegister(REG_LED1_PA, 0x3F);
    writeRegister(REG_LED2_PA, 0x3F);

    return true;
}

void MAX30102::setup() {
    // Configuración predeterminada: limpiar FIFO y ajustar parámetros
    clearFIFO();
    setLEDMode(0x03);           // Red + IR
    setSamplingRate(0x04);      // 100 Hz
    setPulseWidth(0x03);        // 411 µs
    setADCRange(0x02);          // ±4096 nA
    setLEDPulseAmplitudeRed(0x24);
    setLEDPulseAmplitudeIR(0x24);
    wakeUp();                   // Asegurar que el sensor está activo
}

// ---------- Power Control ----------

void MAX30102::shutdown() {
    uint8_t reg = readRegister(REG_MODE_CONFIG);
    reg |= 0x80;
    writeRegister(REG_MODE_CONFIG, reg);
}

void MAX30102::wakeUp() {
    uint8_t reg = readRegister(REG_MODE_CONFIG);
    reg &= ~0x80;
    writeRegister(REG_MODE_CONFIG, reg);
}

// ---------- Configuration Setters ----------

void MAX30102::setLEDMode(uint8_t mode) {
    writeRegister(REG_MODE_CONFIG, mode & 0x07);
}

void MAX30102::setSamplingRate(uint8_t rate) {
    uint8_t reg = readRegister(REG_SPO2_CONFIG);
    reg = (reg & ~(0x07 << 2)) | ((rate & 0x07) << 2);
    writeRegister(REG_SPO2_CONFIG, reg);
}

void MAX30102::setPulseWidth(uint8_t width) {
    uint8_t reg = readRegister(REG_SPO2_CONFIG);
    reg = (reg & ~0x03) | (width & 0x03);
    writeRegister(REG_SPO2_CONFIG, reg);
}

void MAX30102::setADCRange(uint8_t range) {
    uint8_t reg = readRegister(REG_SPO2_CONFIG);
    reg = (reg & ~(0x03 << 5)) | ((range & 0x03) << 5);
    writeRegister(REG_SPO2_CONFIG, reg);
}

void MAX30102::setLEDPulseAmplitudeRed(uint8_t amplitude) {
    writeRegister(REG_LED1_PA, amplitude);
}

void MAX30102::setLEDPulseAmplitudeIR(uint8_t amplitude) {
    writeRegister(REG_LED2_PA, amplitude);
}

// ---------- FIFO Management ----------

void MAX30102::clearFIFO() {
    writeRegister(REG_FIFO_WR_PTR, 0);
    writeRegister(REG_FIFO_RD_PTR, 0);
    writeRegister(REG_FIFO_OVF_COUNTER, 0);
}

uint8_t MAX30102::getWritePtr() {
    return readRegister(REG_FIFO_WR_PTR);
}

uint8_t MAX30102::getReadPtr() {
    return readRegister(REG_FIFO_RD_PTR);
}

uint8_t MAX30102::getFifoCount() {
    uint8_t w = getWritePtr();
    uint8_t r = getReadPtr();
    return (w >= r) ? (w - r) : (w + 32 - r);
}

bool MAX30102::readFIFO(uint32_t &redLED, uint32_t &irLED) {
    uint8_t count = getFifoCount();
    if (count == 0) return false;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t data[6];
        _wire->beginTransmission(_i2caddr);
        _wire->write(REG_FIFO_DATA);
        _wire->endTransmission(false);

        _wire->requestFrom(_i2caddr, (uint8_t)6);
        if (_wire->available() < 6) return false;
        for (int j = 0; j < 6; j++) data[j] = _wire->read();

        uint32_t tmpRed = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        uint32_t tmpIR  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
        tmpRed &= 0x03FFFF;
        tmpIR  &= 0x03FFFF;

        redLED = tmpRed;
        irLED  = tmpIR;
    }
    return true;
}

bool MAX30102::readAllFIFO(std::vector<std::pair<uint32_t, uint32_t>> &outSamples) {
    outSamples.clear();
    uint8_t count = getFifoCount();
    if (count == 0) return false;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t data[6];
        _wire->beginTransmission(_i2caddr);
        _wire->write(REG_FIFO_DATA);
        _wire->endTransmission(false);

        _wire->requestFrom(_i2caddr, (uint8_t)6);
        if (_wire->available() < 6) return false;
        for (int j = 0; j < 6; j++) data[j] = _wire->read();

        uint32_t tmpRed = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        uint32_t tmpIR  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
        tmpRed &= 0x03FFFF;
        tmpIR  &= 0x03FFFF;

        outSamples.emplace_back(tmpRed, tmpIR);
    }
    return true;
}

// ---------- Temperature ----------

void MAX30102::startTemperature() {
    writeRegister(REG_TEMP_CONFIG, 0x01);
}

bool MAX30102::isTemperatureReady() {
    return (readRegister(REG_INTR_STATUS_2) & 0x02) != 0;
}

float MAX30102::readTemperature() {
    int8_t tempInt = (int8_t)readRegister(REG_TEMP_INT);
    uint8_t tempFrac = readRegister(REG_TEMP_FRAC);
    return (float)tempInt + (tempFrac * 0.0625f);
}

// ---------- Device Info ----------

uint8_t MAX30102::getRevisionID() {
    return readRegister(REG_REV_ID);
}

uint8_t MAX30102::getPartID() {
    return readRegister(REG_PART_ID);
}

// ---------- Low-level I2C ----------

uint8_t MAX30102::readRegister(uint8_t reg) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_i2caddr, (uint8_t)1);
    return _wire->available() ? _wire->read() : 0;
}

void MAX30102::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}