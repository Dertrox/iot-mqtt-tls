// — HealthMonitor.h —
#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>
#include <Wire.h>
#include "LIB_SHT31.h"
#include "LIB_MAX30102.h"
#include "COMP_RITMO_CARDIACO.h"
#include "COMP_SPO2.h"

class HealthMonitor {
public:
    void begin();
    void update();  // se llama cada loop

    // Método que llena las variables con el estado actual
    void getCurrentReadings(float &outTemp,
                            float &outHum,
                            float &outBPM,
                            uint8_t &outSpO2,
                            double &outLat,
                            double &outLon,
                            char *outTimestamp,    // buffer de al menos 20 bytes
                            size_t timestampSize);

private:
    // Umbrales y temporizador
    static constexpr float TEMP_ALERT_THRESHOLD     = 37.5f;
    static constexpr float HR_ALERT_HIGH_THRESHOLD  = 120.0f;
    static constexpr float HR_ALERT_LOW_THRESHOLD   = 50.0f;
    static constexpr uint32_t READING_INTERVAL_MS   = 60000;
    uint32_t lastReadingTimestamp = 0;

    // SHT31
    SHT31 sht31;

    // MAX30102
    MAX30102 maxSensor;
    HeartRateProcessor hrProcessor;
    SpO2Processor spo2Processor;
    float dcIR = 0.0f, dcRed = 0.0f;
    bool fingerPresent = false;
    float lastValidBPM = 0.0f;

    // GPS
    static const int RXPin = 16;
    static const int TXPin = 17;
    static const uint32_t GPSBaud = 9600;
    static const int UTC_OFFSET_SECONDS = -5 * 3600;
    TinyGPSPlus gps;
    HardwareSerial GPS_Serial = HardwareSerial(2);

    // Métodos internos
    void processMAX30102();
    void readGPS();
};

#endif
