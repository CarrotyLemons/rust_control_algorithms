#![no_std]

pub enum Clamping 
{
    None,
    LowerLimit(f64),
    UpperLimit(f64),
    BothLimits(f64, f64)
}

impl Clamping
{
    pub fn exceeded(&self, value: f64) -> bool{
        match &self {
            Clamping::None => false,
            Clamping::UpperLimit(upper_limit) => {
                value > *upper_limit
            }
            Clamping::LowerLimit(lower_limit) => {
                value < *lower_limit
            }
            Clamping::BothLimits(lower_limit, upper_limit) => {
                (value < *lower_limit) || (value > *upper_limit)
            }
        }
    }
}

pub struct Pid
{
    pub kp: f64,
    pub kd: f64,
    pub ki: f64,
    pub target: f64, // Set point
    pub process_variable: f64, // The current output of the model
    pub cumulative_error: f64, // Used for I term calculations
    pub previous_measurement: f64, // Used for D term calculations
    pub clamping: Clamping, // Used for clamping in I term calculations
}

impl Pid {
    // Returns a blank Pid struct with default values
    pub fn blank() -> Self{
        Pid {
            kp: 0.0,
            kd: 0.0,
            ki: 0.0,
            target: 0.0, // PID target value
            process_variable: 0.0, // Current model output
            cumulative_error: 0.0, // Integral of error
            previous_measurement: 0.0, // Used to calculate the derivative of change
            clamping: Clamping::None,
        }
    }

    pub fn step(&mut self, measured: f64, time_step: f64) -> f64 {
        let error = self.target - measured;
        
        // Proportional calculation
        let mut change_output = self.kp * error;

        // Integral calculation with clamping windup control
        if !self.clamping.exceeded(self.cumulative_error) {
            self.cumulative_error += error * time_step;
            change_output += self.ki * self.cumulative_error;
        }

        // Derivative calculation
        // Takes the derivative of the process variable to prevent issues when the target changes.
        change_output += self.kd * ((measured - self.previous_measurement) / time_step);
        self.previous_measurement = measured;

        self.process_variable += change_output;

        self.process_variable
    }
}