/**
 * @class EngineController
 *
 * @brief Manages the control of up to four engines using electronic speed controllers (ESCs).
 *
 * The EngineController class allows precise management of engine speeds through ESCs,
 * utilizing PWM (Pulse Width Modulation). It supports setting individual engine speeds
 * as well as synchronous control of all engines. This class is suitable for applications
 * such as drones, where precise and individual motor control is required.
 *
 * @details
 * Initialization of the class requires the specification of the ESC control pins,
 * PWM frequency, PWM resolution, and the pulse width boundaries that correspond to
 * the minimum and maximum throttle settings.
 *
 * Functions:
 * - Initialize ESCs to prepare them for signal input.
 * - Convert a desired pulse width into a corresponding duty cycle value.
 * - Stop all engines either immediately or with a delay.
 * - Set the speed of an individual engine or all engines together.
 *
 * Example:
 * ```cpp
 * std::array<const int, 4> escPins = {32, 33, 34, 35};
 * EngineController engineController(escPins, 20000, 12, 1000, 2000);
 * engineController.initializeESC();
 * engineController.setEngineSpeed(1, 50); // Set engine 1 to 50% throttle
 * ```
 *
 * @param escPins Array of pin numbers connected to the ESC signal inputs.
 * @param frequency PWM frequency for ESC signaling.
 * @param resolution Bit resolution for PWM signal.
 * @param minPulseWidth Minimum pulse width corresponding to 0% throttle.
 * @param maxPulseWidth Maximum pulse width corresponding to 100% throttle.
 */
class EngineController {
public:
    /**
     * @brief Constructs an EngineController with given parameters.
     */
    EngineController(std::array<const int, 4> &escPins, const int frequency, const int resolution, const int minPulseWidth, const int maxPulseWidth) :
        escPins(escPins), frequency(frequency), resolution(resolution), minPulseWidth(minPulseWidth), maxPulseWidth(maxPulseWidth) {}

    /**
     * @brief Initializes ESCs by setting up PWM channels and attaching them to the specified pins.
     *
     * This method must be called before setting engine speeds to ensure that the PWM generators
     * are correctly configured and linked to the correct output pins.
     */
    void initializeESC() {
        for (int i = 0; i <= 3; i++) {
            ledcSetup(i, frequency, resolution);
            ledcAttachPin(escPins[i], i);
        }
    }

    /**
     * @brief Converts a pulse width in microseconds to a corresponding duty cycle value.
     *
     * @param pulseWidth The pulse width in microseconds.
     * @return The calculated duty cycle value based on the current resolution.
     */
    float pulseWidthToDutyCycle(float pulseWidth) {
        float maxDutyCycle = pow(2, resolution) - 1;
        return map(pulseWidth, 0, 20000, 0, maxDutyCycle);
    }

    /**
     * @brief Stops all engines by setting their throttle to the minimum pulse width.
     *
     * @param delayMilliseconds Time in milliseconds to delay after stopping the engines.
     * This is useful for ensuring all operations requiring the stop to complete.
     */
    void stopEngines(int delayMilliseconds) {
        for (int i = 0; i <= 3; i++) {
            ledcWrite(i , pulseWidthToDutyCycle(minPulseWidth));
        }
        Serial.print("Engines stopped");

        delay(delayMilliseconds);
    }

    /**
     * @brief Sets the speed (throttle) of specified engines.
     *
     * @param engineNumber Engine identifier (1-4 for individual engines, 5 for all engines).
     * @param throttle Throttle percentage from 0% to 100%.
     *
     * If an invalid engine number or throttle percentage is specified, an error message is printed.
     */
    void setEngineSpeed(int engineNumber, float throttle) {
        if (engineNumber < 1 || engineNumber > 5 ) {
            Serial.println("Engine number should be 1-5, 1-4 individual, 5 all.");
            return;
        }
        if (throttle < 0 || throttle > 100 ) {
            Serial.println("Throttle should be 0-100.");
            return;
        }

        if (engineNumber >= 1 && engineNumber <= 4) {
            float pulseWidth = map(throttle, 0, 100, minPulseWidth, maxPulseWidth);
            ledcWrite(engineNumber - 1, pulseWidthToDutyCycle(pulseWidth));
            //Serial.println("Engine " + String(engineNumber) + ": Throttle set to " + String(throttle) + "%");
        }

        else if (engineNumber == 5) {
            for(int i = 0; i <= 3; i++) {
                float pulseWidth = map(throttle, 0, 100, minPulseWidth, maxPulseWidth);
                ledcWrite(i, pulseWidthToDutyCycle(pulseWidth));
            }
            Serial.println("Engines set to " + String(throttle) + "% throttle");
        }
    }

private:
    std::array<const int, 4> escPins; ///< Pins connected to ESCs.
    const int frequency;              ///< PWM frequency.
    const int resolution;             ///< PWM resolution.
    const int minPulseWidth;          ///< Minimum pulse width (0% throttle).
    const int maxPulseWidth;          ///< Maximum pulse width (100% throttle).
};