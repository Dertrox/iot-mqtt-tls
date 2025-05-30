#ifndef _SHT31_H_
#define _SHT31_H_

#include <Arduino.h>
#include <Wire.h>

class SHT31 {
public:
    // Direcciones I2C según pin ADDR
    static constexpr uint8_t ADDR_0x44 = 0x44; // ADDR→GND
    static constexpr uint8_t ADDR_0x45 = 0x45; // ADDR→VDD

    // Repetibilidad para mediciones single-shot
    enum Repeatability : uint16_t {
        REP_LOW    = 0x2C10, // baja repetibilidad
        REP_MEDIUM = 0x2C0D, // repetibilidad media
        REP_HIGH   = 0x2C06  // alta repetibilidad
    };

    // Clock-stretching
    enum ClockStretch : uint16_t {
        CS_ENABLE  = 0x0000, // habilitado
        CS_DISABLE = 0x2400  // deshabilitado
    };

    // Códigos de error posibles
    enum ErrorCode {
        ERROR_NONE,
        ERROR_NOT_CONNECTED,
        ERROR_CRC,
        ERROR_TIMEOUT,
        ERROR_UNKNOWN
    };

    //(bus I2C y dirección opcionales)
    SHT31(TwoWire &wire = Wire, uint8_t addr = ADDR_0x44);

    // Inicializa comunicación I2C
    bool begin();

    // Lee temperatura (°C) y humedad (% RH)
    // Devuelve true si la lectura es exitosa
    bool read(float &temperature, float &humidity,
              Repeatability rep = REP_HIGH,
              ClockStretch cs = CS_ENABLE);

    // Lee solo temperatura
    float readTemperature(Repeatability rep = REP_HIGH,
                          ClockStretch cs = CS_ENABLE);

    // Lee solo humedad
    float readHumidity(Repeatability rep = REP_HIGH,
                       ClockStretch cs = CS_ENABLE);

    // Soft-reset (comando 0x30A2)
    bool softReset();

    // Limpia registro de estado (comando 0x3041)
    bool clearStatus();

    // Devuelve el último código de error
    ErrorCode getError() const;

    // Devuelve mensaje de error asociado
    const char* getErrorMessage() const;

private:
    TwoWire &_wire;
    uint8_t  _addr;
    ErrorCode _error;

    // Conversión raw -> valor físico
    static constexpr float TEMP_OFFSET = -45.0f;
    static constexpr float TEMP_SCALE  = 175.0f;
    static constexpr float HUM_SCALE   = 100.0f;

    // Envía comando de 16 bits
    bool sendCommand(uint16_t cmd);

    // Lee datos crudos y verifica CRC
    bool readRaw(uint16_t &rawTemp, uint16_t &rawHum);

    // CRC-8 polinomio 0x31, init 0xFF
    static uint8_t crc8(const uint8_t *data, uint8_t len);
};

#endif // _SHT31_H_