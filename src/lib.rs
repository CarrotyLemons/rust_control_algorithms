#![no_std]
use core::marker::PhantomData;

use rust_helper_tools::Floats;

pub enum Clamping<T> 
where
    T: Floats,
{
    None,
    LowerLimit(T),
    UpperLimit(T),
    BothLimits(T, T)
}

impl<T> Clamping<T>
where
    T: Floats,
{
    pub fn exceeded(&self, value: T) -> bool{
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
    pub output: T,
    pub clamping: Clamping<T>,
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
            output: T::default(),
            clamping: Clamping::None,
            _marker: PhantomData,
        }
    }

    pub fn step(&mut self, measured: T, time_step: T) -> T {
        let error = self.target - measured;
        let mut result = self.kp * error + self.kd * ((error - self.previous_error) / time_step);

        if !self.clamping.exceeded(self.output) {
            self.cumulative_error += error * time_step;
            result += self.ki * self.cumulative_error;
        }

        self.previous_error = error;
        self.output = result;

        result
    }
}