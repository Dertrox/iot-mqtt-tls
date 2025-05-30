#include "LIB_SHT31.h"

// Constructor
SHT31::SHT31(TwoWire &wire, uint8_t addr)
  : _wire(wire), _addr(addr), _error(ERROR_NONE) {}

// Inicia I2C y hace soft reset para verificar conexión
bool SHT31::begin() {
    _wire.begin();
    if (!softReset()) {
        _error = ERROR_NOT_CONNECTED;
        return false;
    }
    _error = ERROR_NONE;
    return true;
}

// Lectura de temperatura y humedad (usa readRaw y conversión)
bool SHT31::read(float &temperature, float &humidity,
                  Repeatability /*rep*/, ClockStretch /*cs*/) {
    uint16_t rawT, rawH;
    if (!readRaw(rawT, rawH)) {
        return false;
    }
    temperature = TEMP_OFFSET + TEMP_SCALE * rawT / 65535.0f;
    humidity    = HUM_SCALE   * rawH / 65535.0f;
    return true;
}

// Lee solo temperatura
float SHT31::readTemperature(Repeatability rep, ClockStretch cs) {
    float h;
    float t;
    if (read(t, h, rep, cs)) return t;
    return NAN;
}

// Lee solo humedad
float SHT31::readHumidity(Repeatability rep, ClockStretch cs) {
    float t;
    float h;
    if (read(t, h, rep, cs)) return h;
    return NAN;
}

// Soft reset (comando 0x30A2)
bool SHT31::softReset() {
    if (!sendCommand(0x30A2)) {
        _error = ERROR_NOT_CONNECTED;
        return false;
    }
    delay(1);
    _error = ERROR_NONE;
    return true;
}

// Limpia registro de estado (comando 0x3041)
bool SHT31::clearStatus() {
    if (!sendCommand(0x3041)) {
        _error = ERROR_NOT_CONNECTED;
        return false;
    }
    _error = ERROR_NONE;
    return true;
}

// Devuelve último código de error
SHT31::ErrorCode SHT31::getError() const {
    return _error;
}

// Mensaje según código de error
const char* SHT31::getErrorMessage() const {
    switch (_error) {
        case ERROR_NONE:          return "No error";
        case ERROR_NOT_CONNECTED: return "Sensor no conectado";
        case ERROR_CRC:           return "Error de CRC";
        case ERROR_TIMEOUT:       return "Timeout I2C";
        default:                  return "Error desconocido";
    }
}

// Envía comando de 16 bits al sensor
bool SHT31::sendCommand(uint16_t cmd) {
    _wire.beginTransmission(_addr);
    _wire.write(cmd >> 8);
    _wire.write(cmd & 0xFF);
    return (_wire.endTransmission() == 0);
}

// Lee datos crudos y verifica CRC
bool SHT31::readRaw(uint16_t &rawTemp, uint16_t &rawHum) {
    _error = ERROR_NONE;
    if (!sendCommand(0x2C06)) { // High repeatability + CRC
        _error = ERROR_NOT_CONNECTED;
        return false;
    }
    delay(15); 

    if (_wire.requestFrom(_addr, (uint8_t)6) < 6) {
        _error = ERROR_TIMEOUT;
        return false;
    }
    uint8_t buf[6];
    for (int i = 0; i < 6; i++) buf[i] = _wire.read();

    // Verificar CRC de temperatura y humedad
    if (crc8(buf, 2) != buf[2] || crc8(buf + 3, 2) != buf[5]) {
        _error = ERROR_CRC;
        return false;
    }

    rawTemp = (uint16_t(buf[0]) << 8) | buf[1];
    rawHum  = (uint16_t(buf[3]) << 8) | buf[4];
    return true;
}

// CRC-8 polinomio 0x31, init 0xFF
uint8_t SHT31::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}