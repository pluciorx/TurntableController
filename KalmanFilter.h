#pragma once
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    // Constructor to initialize the Kalman filter
    KalmanFilter(double processNoise, double measurementNoise, double estimatedError, double initialValue);

    // Function to update the Kalman filter with a new measurement
    double update(double measurement);

private:
    double Q; // Process noise covariance
    double R; // Measurement noise covariance
    double P; // Estimation error covariance
    double K; // Kalman gain
    double X; // Estimated value
};

#endif

