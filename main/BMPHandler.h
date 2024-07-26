#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define HSPI_CS_PIN 15
#define HSPI_SCK_PIN 14
#define HSPI_SDI_PIN 12
#define HSPI_SDO_PIN 13

Adafruit_BMP3XX bmp;

class BMPHandler {
public:
    BMPHandler() {}

    void initializeBMP() {
        Wire.begin();
        bmp.begin_SPI(HSPI_CS_PIN, HSPI_SCK_PIN, HSPI_SDO_PIN, HSPI_SDI_PIN);

        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }

    float getAltitude() {
        bmp.preformReading();
        float pressure = bmp.pressure;

        const float p0 = 101325.0; // Standard atmospheric pressure at sea level in Pa
        float altitude = 44330.0 * (1.0 - pow(pressure / P0, 0.1903));
        return altitude;
    }
}
