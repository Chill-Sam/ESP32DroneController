#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

#define HSPI_CS_PIN 15
#define HSPI_SCK_PIN 14
#define HSPI_SDI_PIN 12
#define HSPI_SDO_PIN 13

Adafruit_DPS310 dps;

class DPSHandler {
public:
    DPSHandler() {}

    void initializeDPS() {
        Wire.begin();
        dps.begin_SPI(HSPI_CS_PIN, HSPI_SCK_PIN, HSPI_SDO_PIN, HSPI_SDI_PIN);
        
        dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
        dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    }

    float getAltitude() {
        sensors_event_t temp_event, pressure_event;
        dps.getEvents(&temp_event, &pressure_event);
        float pressure = pressure_event.pressure;

        const float p0 = 101325.0; // Standard atmospheric pressure at sea level in Pa
        float altitude = 44330.0 * (1.0 - pow(pressure / P0, 0.1903));
        return altitude;
    }
}
