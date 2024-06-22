/**
 * @class KalmanFilter
 *
 * @brief Implements a basic Kalman filter for single-variable state estimation.
 *
 * This class provides a simple implementation of a Kalman filter, designed for estimating
 * the state of a single variable system. It is used in scenarios where you want to
 * estimate a system state that evolves over time with inherent uncertainties from
 * noisy measurements and predictions.
 *
 * @details
 * The KalmanFilter class is initialized with parameters that define the behavior of the
 * system and the characteristics of the noise in the measurements and predictions. The
 * filter updates its estimates through two primary operations: predict and update.
 *
 * Usage:
 * 1. Create an instance of KalmanFilter with the desired system parameters.
 * 2. Use the predict() method to advance the state estimation based on the model.
 * 3. Use the update() method with a new measurement to refine the state estimation.
 *
 * Example:
 * ```cpp
 * KalmanFilter kf(0.1, 0.1, 1.0, 0.0, 1.0);
 * kf.predict();
 * kf.update(1.2, kf.getState() - 1.2);
 * float state = kf.getState();
 * ```
 */
class KalmanFilter {
public:
    /**
     * @brief Constructs a KalmanFilter with initial parameters.
     * @param R Measurement noise covariance.
     * @param Q Process noise covariance.
     * @param A State transition matrix (scalar since it's a single-variable filter).
     * @param B Control input matrix (scalar, not used in this implementation).
     * @param H Measurement matrix (scalar).
     */
    KalmanFilter(float R, float Q, float A, float B, float H) :
        R_(R), Q_(Q), A_(A), B_(B), H_(H), x_est_(0), P_(1) {}

    /**
     * @brief Predicts the next state of the system using the model's dynamics.
     *
     * This method advances the state estimate based on the system's model defined by the
     * state transition matrix A_. It adjusts the estimation error covariance accordingly.
     */
    void predict() {
        x_est_ = A_ * x_est_;
        P_ = A_ * P_ * A_ + Q_;
    }

    /**
     * @brief Updates the estimated state based on new measurement.
     *
     * This method refines the state estimate using a new measurement. It computes the
     * Kalman gain, updates the state estimate with the measurement residual, and
     * adjusts the estimation error covariance.
     * 
     * @param z Measurement value.
     * @param residual The difference between measured and estimated values.
     */
    void update(float z, float residual) {
        float K = P_ * H_ / (H_ * P_ * H_ + R_);
        x_est_ = x_est_ + K * residual;
        P_ = (1 - K * H_) * P_;
    }

    /**
     * @brief Retrieves the current state estimate.
     * 
     * @return Current state estimate as a float.
     */
    float getState() const {
        return x_est_;
    }

private:
    float R_;   ///< Measurement noise covariance
    float Q_;   ///< Process noise covariance
    float A_;   ///< State transition matrix
    float B_;   ///< Control input matrix (not used in this implementation)
    float H_;   ///< Measurement matrix
    float x_est_;   ///< Current state estimate
    float P_;   ///< Estimation error covariance
};