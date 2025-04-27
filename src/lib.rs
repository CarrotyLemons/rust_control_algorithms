#![no_std]
use rust_helper_tools::Floats;

pub struct Pid<T>
where
    T: Floats,
{
    pub kp: T,
    pub kd: T,
    pub ki: T,
    pub target: T,
    pub cumulative_error: T,
    pub previous_error: T,
}

impl<T> Pid<T>
where
    T: Floats,
{
    pub fn new(kp: T, kd: T, ki: T, target: T, cumulative_error: T, previous_error: T) -> Self {
        Pid {
            kp,
            kd,
            ki,
            target,
            cumulative_error,
            previous_error,
        }
    }

    pub fn step(&mut self, measured: T, time_step: T) -> T {
        let error = self.target - measured;
        self.cumulative_error += error * time_step;
        let result = self.kp * error
            + self.kd * ((error - self.previous_error) / time_step)
            + self.ki * self.cumulative_error;
        self.previous_error = error;

        result
    }
}