class KalmanFilter:
    def __init__(self, initial_estimate, initial_uncertainty, process_noise, measurement_noise):
        # Store the initial estimate
        self.init_est = initial_estimate
        # Store the initial uncertainty
        self.init_uncertainty = initial_uncertainty
        # Store process noise Q
        self.process_noise = process_noise
        # Store measurement noise R
        self.measurement_noise = measurement_noise
        self.state_pred = 0
        self.uncertainty_pred = 0
        self.uncertainty = initial_uncertainty
        self.state = initial_estimate
        self.K = 0

    def predict(self):
        # Predict the state
        self.state_pred = self.state
        # Predict the uncertainty
        self.uncertainty_pred = self.uncertainty + self.process_noise

    def update(self, measurement):
        # Calculate Kalman gain
        self.K = self.uncertainty_pred/(self.uncertainty_pred + self.measurement_noise)
        # Correct the estimate using the measurement
        self.state = self.state_pred + self.K*(measurement-self.state_pred)
        # Update the uncertainty
        self.uncertainty = (1-self.K) * self.uncertainty_pred