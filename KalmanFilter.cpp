#include "KalmanFilter.h"

// Constructor to initialize the Kalman filter
KalmanFilter::KalmanFilter(double processNoise, double measurementNoise, double estimatedError, double initialValue) {
    Q = processNoise;
    R = measurementNoise;
    P = estimatedError;
    X = initialValue;
}

// Function to update the Kalman filter with a new measurement
double KalmanFilter::update(double measurement) {
    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;
    return X;
}