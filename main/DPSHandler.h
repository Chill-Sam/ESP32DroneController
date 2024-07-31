/**
 * @file DPSHandler.h
 *
 * @brief Interfaces with the DPS310 sensor to read pressure and temperature data.
 *
 * This class encapsulates methods to initialize the DPS310 sensor, read its pressure and temperature data,
 * and handle SPI communication with the sensor.
 *
 * Dependencies:
 * - SPI.h: Handles SPI communication.
 * - math.h: Provides mathematical constants and functions.
 *
 * Example Usage:
 * ```cpp
 * DPSHandler dps;
 * dps.initializeDPS();
 * float pressure = dps.getPressure();
 * float altitude = dps.getAltitude();
 * Serial.print("Pressure: ");
 * Serial.println(pressure);
 * Serial.print("Altitude: ");
 * Serial.println(altitude);
 * ```
 */

#include <SPI.h>
#include <math.h>

// Define the pin numbers for the DPS310 connection
#define DPS_CS_PIN 15
#define SCK_PIN 18  ///< Serial Clock pin
#define SDI_PIN 19  ///< Serial Data In pin
#define SDO_PIN 23  ///< Serial Data Out pin

// Define register addresses and constants for the DPS310
#define MEAS_CFG 0x08
#define PRS_CFG 0x06
#define TMP_CFG 0x07
#define PROD_ID_REG 0x0D

#define PSR_B2 0x00
#define PSR_B1 0x01
#define PSR_B0 0x02
#define TMP_B2 0x03
#define TMP_B1 0x04
#define TMP_B0 0x05

#define COEF_START 0x10
#define COEF_LEN 18

#define kT 1040384
#define kP 1040384

/**
 * @brief The DPSHandler class handles communication with the DPS310 sensor.
 */
class DPSHandler {
public:
  /**
   * @brief Constructor for DPSHandler.
   */
  DPSHandler() {}

  /**
   * @brief Initializes the DPS310 sensor and verifies communication.
   */
  void initializeDPS() {
    // Start SPI communication
    SPI.begin(SCK_PIN, SDI_PIN, SDO_PIN, DPS_CS_PIN);

    pinMode(DPS_CS_PIN, OUTPUT);
    digitalWrite(DPS_CS_PIN, HIGH);

    delay(100);

    // Read and print the Product ID to verify communication
    uint8_t prodId = readRegister(PROD_ID_REG);
    Serial.print("Product ID: 0x");
    Serial.println(prodId, HEX);

    // Check if the product ID is correct
    if (prodId == 0x10) {
      Serial.println("DPS310 communication successful.");
    } else {
      Serial.println("Failed to communicate with DPS310.");
      return;  // Stop further initialization if communication fails
    }

    uint8_t meas_cfg = readRegister(MEAS_CFG);
    meas_cfg |= 0x07;
    writeRegister(MEAS_CFG, meas_cfg);

    meas_cfg = readRegister(MEAS_CFG);
    Serial.print("MEAS_CFG register: 0x");
    Serial.println(meas_cfg, HEX);

    writeRegister(PRS_CFG, 0x66);  // Set measurement rate to 64Hz and 64x oversampling
    writeRegister(TMP_CFG, 0x66);  // Set measurement rate to 64Hz and 64x oversampling

    delay(100);

    while (!(readRegister(MEAS_CFG) & (1 << 6))) {
      Serial.println("DPS310 is not ready yet.");
      Serial.println(meas_cfg, HEX);
    }

    delay(100);

    while (!(readRegister(MEAS_CFG) & (1 << 7))) {
      Serial.println("Coefficients are not available yet.");
    }

    getCoefficients();

    Serial.println("DPS310 Initialization Done");
  }

  /**
   * @brief Gets the current pressure reading from the DPS310.
   *
   * @return The compensated pressure value.
   */
  float getPressure() {
    uint8_t tmp_b2 = readRegister(TMP_B2);
    uint8_t tmp_b1 = readRegister(TMP_B1);
    uint8_t tmp_b0 = readRegister(TMP_B0);

    int32_t Traw = twosComplement(((tmp_b2 << 16) | (tmp_b1 << 8) | tmp_b0), 24);

    uint8_t psr_b2 = readRegister(PSR_B2);
    uint8_t psr_b1 = readRegister(PSR_B1);
    uint8_t psr_b0 = readRegister(PSR_B0);

    int32_t Praw = twosComplement(((psr_b2 << 16) | (psr_b1 << 8) | psr_b0), 24);

    float Traw_sc = (float)Traw / kT;
    float Praw_sc = (float)Praw / kP;

    float Pcomp = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);

    return Pcomp;
  }

  /**
   * @brief Gets the current altitude based on the pressure reading from the DPS310.
   *
   * @return The calculated altitude value.
   */
  float getAltitude() {
    float pressure = getPressure();
    float altitude = convertToAltitude(pressure);

    return altitude;
  }

  /**
   * @brief Reads the calibration coefficients from the DPS310 sensor.
   */
  void getCoefficients() {
    uint8_t coeffs[COEF_LEN];

    for (int i = 0; i < COEF_LEN; i++) {
      coeffs[i] = readRegister(COEF_START + i);
    }

    c0 = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    c0 = twosComplement(c0, 12);

    c1 = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);

    c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
         (((uint32_t)coeffs[5] >> 4) & 0x0F);
    c00 = twosComplement(c00, 20);

    c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
         (uint32_t)coeffs[7];
    c10 = twosComplement(c10, 20);

    c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
    c11 = twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
    c20 = twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
    c21 = twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
    c30 = twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);
  }

private:
  int16_t c0, c1;
  int32_t c00, c10, c01, c11, c20, c21, c30;

  /**
   * @brief Writes a byte of data to a specific register of the DPS310.
   *
   * @param reg The register address to write to.
   * @param value The data byte to write to the register.
   */
  void writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));  ///< Adjust SPI settings if needed
    digitalWrite(DPS_CS_PIN, LOW);                                    ///< Select the slave device
    SPI.transfer(reg);                                                ///< Register address
    SPI.transfer(value);                                              ///< Register value
    digitalWrite(DPS_CS_PIN, HIGH);                                   ///< Deselect the slave device
    SPI.endTransaction();
  }

  /**
   * @brief Reads a byte of data from a specific register of the DPS310.
   *
   * @param reg The register address to read from.
   * @return The byte value read from the register.
   */
  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));  ///< Adjust SPI settings if needed
    digitalWrite(DPS_CS_PIN, LOW);                                    ///< Select the slave device
    SPI.transfer(reg | 0x80);                                         ///< Register address with read flag
    value = SPI.transfer(0x00);                                       ///< Dummy write to read the register value
    digitalWrite(DPS_CS_PIN, HIGH);                                   ///< Deselect the slave device
    SPI.endTransaction();
    return value;
  }

  /**
   * @brief Converts a value from two's complement format.
   *
   * @param val The value to convert.
   * @param bits The number of bits in the value.
   * @return The converted value.
   */
  int32_t twosComplement(int32_t val, uint8_t bits) {
    if (val & ((uint32_t)1 << (bits - 1))) {
      val -= (uint32_t)1 << bits;
    }
    return val;
  }

  /**
   * @brief Converts pressure to altitude using the barometric formula.
   *
   * @param pressure The pressure value to convert.
   * @return The calculated altitude.
   */
  float convertToAltitude(float pressure) {
    float altitude = 44330 * (1.0 - pow((pressure / 100) / 1013.25, 0.1903));
    return altitude;
  }
};