// — HealthMonitor.cpp —
#include "HealthMonitor.h"

void HealthMonitor::begin() {
    Wire.begin();
    Serial.println("Inicializando sensores...");

    if (!sht31.begin()) {
        Serial.println("Error al iniciar SHT31: " + String(sht31.getErrorMessage()));
        while (true) delay(1000);
    }
    Serial.println("SHT31 ok.");

    if (!maxSensor.begin()) {
        Serial.println("Error: MAX30102 no encontrado.");
        while (true) delay(100);
    }
    maxSensor.setup();
    hrProcessor.reset();
    spo2Processor.reset();
    Serial.println("MAX30102 ok.");

    GPS_Serial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Serial.println("GPS ok.");
}

void HealthMonitor::update() {
    uint32_t now = millis();
    if (now - lastReadingTimestamp < READING_INTERVAL_MS) {
        readGPS();
        processMAX30102();
        return;
    }
    lastReadingTimestamp = now;

    // Cada minuto: lee SHT31, MAX30102 y GPS (ya leído en readGPS())
    float temp, hum;
    bool okTemp = sht31.read(temp, hum);

    processMAX30102();
    readGPS();

    // Solo imprimimos por Serial (puedes comentar si no lo necesitas)
    float bpm = lastValidBPM;
    uint8_t spO2 = spo2Processor.getSpO2();
    char horario[20];
    sprintf(horario, "%02d/%02d/%04d %02d:%02d:%02d",
            day(), month(), year(), hour(), minute(), second());

    if (okTemp && (temp >= TEMP_ALERT_THRESHOLD || bpm >= HR_ALERT_HIGH_THRESHOLD || (bpm>0 && bpm <= HR_ALERT_LOW_THRESHOLD))) {
        Serial.println("*** ALERTA ***");
        if (okTemp && temp >= TEMP_ALERT_THRESHOLD) Serial.printf("Temp alta: %.2f°C\n", temp);
        if (bpm >= HR_ALERT_HIGH_THRESHOLD || (bpm>0 && bpm <= HR_ALERT_LOW_THRESHOLD)) 
            Serial.printf("BPM anómalo: %.1f\n", bpm);
    } else {
        Serial.println("Estable.");
        if (okTemp) Serial.printf("Temp: %.2f°C, Hum: %.2f%%, ", temp, hum);
        else        Serial.print("Temp: N/A, Hum: N/A, ");
        Serial.printf("BPM: %.1f, SpO2: %u%%\n", bpm, spO2);
    }
    Serial.printf("Hora: %s\n", horario);
    Serial.printf("GPS: Lat %.6f, Lon %.6f\n", gps.location.lat(), gps.location.lng());
    Serial.println("--------------------------");
}

// Este método simplemente “extrae” valores actuales y arma el timestamp
void HealthMonitor::getCurrentReadings(float &outTemp,
                                      float &outHum,
                                      float &outBPM,
                                      uint8_t &outSpO2,
                                      double &outLat,
                                      double &outLon,
                                      char *outTimestamp,
                                      size_t timestampSize) {
    // SHT31
    float t = NAN, h = NAN;
    if (sht31.read(t, h)) {
        outTemp = t;
        outHum = h;
    } else {
        outTemp = NAN;
        outHum = NAN;
    }

    // MAX30102 (ya procesado en processMAX30102 cuando hubo muestras)
    outBPM = lastValidBPM;
    outSpO2 = spo2Processor.getSpO2();

    // GPS
    if (gps.location.isValid()) {
        outLat = gps.location.lat();
        outLon = gps.location.lng();
    } else {
        outLat = 0.0;
        outLon = 0.0;
    }

    // Timestamp usando TimeLib (ya fue seteado en readGPS())
    snprintf(outTimestamp, timestampSize, "%02d/%02d/%04d %02d:%02d:%02d",
             day(), month(), year(), hour(), minute(), second());
}

void HealthMonitor::processMAX30102() {
    std::vector<std::pair<uint32_t, uint32_t>> samples;
    if (!maxSensor.readAllFIFO(samples)) return;

    uint32_t t = millis();
    for (auto &p : samples) {
        uint32_t rawRed = p.first;
        uint32_t rawIR  = p.second;

        if (!fingerPresent && rawIR > 30000) {
            fingerPresent = true;
            hrProcessor.reset(); spo2Processor.reset();
        } else if (fingerPresent && rawIR < 20000) {
            fingerPresent = false;
            hrProcessor.reset(); spo2Processor.reset();
            return;
        }
        if (!fingerPresent) return;

        dcIR  = 0.95f * dcIR  + 0.05f * rawIR;
        dcRed = 0.95f * dcRed + 0.05f * rawRed;
        float acIR  = float(rawIR)  - dcIR;
        float acRed = float(rawRed) - dcRed;

        bool beat = hrProcessor.update(acIR, t);
        spo2Processor.update(acIR, acRed, beat);
        float rawBPM = hrProcessor.getBPM();
        if (rawBPM >= 40.0f && rawBPM <= 180.0f) lastValidBPM = rawBPM;
    }
}

void HealthMonitor::readGPS() {
    while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());
    if (gps.date.isValid() && gps.time.isValid()) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
                gps.date.day(), gps.date.month(), gps.date.year());
        adjustTime(UTC_OFFSET_SECONDS);
    }
}
