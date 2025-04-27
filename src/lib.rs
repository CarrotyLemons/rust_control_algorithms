#![no_std]
use core::marker::PhantomData;

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
    pub _marker: PhantomData<T>,
}

impl<T> Pid<T>
where
    T: Floats,
{
    //Returns a blank Pid struct with default values
    pub fn blank() -> Self{
        Pid {
            kp: T::default(),
            kd: T::default(),
            ki: T::default(),
            target: T::default(),
            cumulative_error: T::default(),
            previous_error: T::default(),
            _marker: PhantomData,
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