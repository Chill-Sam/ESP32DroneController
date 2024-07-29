#include <SPI.h>

#define DPS_CS_PIN 15
#define SCK_PIN 18  ///< Serial Clock pin
#define SDI_PIN 19  ///< Serial Data In pin
#define SDO_PIN 23  ///< Serial Data Out pin

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

int16_t c0, c1;
int32_t c00, c10, c01, c11, c20, c21, c30;

class DPSHandler {
public:
  DPSHandler() {}

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

    uint8_t meas_cfg = readRegister(MEAS_CFG);
    Serial.print("MEAS_CFG register: 0x");
    Serial.println(meas_cfg, HEX);

    // Check if the product ID is correct
    if (prodId == 0x10) {
      Serial.println("DPS310 communication successful.");
    } else {
      Serial.println("Failed to communicate with DPS310.");
      return;  // Stop further initialization if communication fails
    }

    meas_cfg = readRegister(MEAS_CFG);
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

  int32_t getPressure() {
    uint8_t psr_b2 = readRegister(PSR_B2);
    uint8_t psr_b1 = readRegister(PSR_B1);
    uint8_t psr_b0 = readRegister(PSR_B0);

    int32_t Praw = (psr_b2 << 16) | (psr_b1 << 8) | psr_b0;

    uint8_t tmp_b2 = readRegister(TMP_B2);
    uint8_t tmp_b1 = readRegister(TMP_B1);
    uint8_t tmp_b0 = readRegister(TMP_B0);

    int32_t Traw = (tmp_b2 << 16) | (tmp_b1 << 8) | tmp_b0;

    float Traw_sc = Traw / kT;
    float Praw_sc = Praw / kP;

    float Pcomp = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);

    return Praw;
  }

  void getCoefficients() {
    uint8_t coef[COEF_LEN];

    for (int i = 0; i < COEF_LEN; i++) {
      coef[i] = readRegister(COEF_START + i);
    }

  }

private:
  /**
   * @brief Writes a byte of data to a specific register of the MPU6000.
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
   * @brief Reads a byte of data from a specific register of the MPU6000.
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
};