#![no_std]
use rust_helper_tools::Floats;

pub struct Pid<T>
where
    T: Floats,
{
    kp: T,
    kd: T,
    ki: T,
    target: T,
    cumulative_error: T,
    previous_error: T,
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

// More flexible assertion macro
#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use rust_helper_tools::assert_flexible;

    #[test]
    // Test the use of an exclusively proportional PID
    fn test_proportional_pid() {
        let mut controller = Pid::new(0.2 as f64, 0.0, 0.0, 15.2, 0.0, 0.0);
        let result = controller.step(3.5 as f64, 0.01);

        assert_eq!(result, 0.2 * (15.2 - 3.5))
    }

    #[test]
    // Test the use of an exclusively derivative PID
    fn test_derivative_pid() {
        let mut controller = Pid::new(0 as f64, 2.2, 0.0, 12.0, 0.0, -0.1);
        let result = controller.step(0.05 as f64, 0.01);

        assert_flexible!(result, 2.2 * ((12.0 - 0.05) + 0.1) / (0.01), 0.01)
    }

    #[test]
    // Test the use of an exclusively integral PID
    fn test_integral_pid() {
        let mut controller = Pid::new(0 as f64, 0.0, 0.5, 6.0, 3.0, 0.0);
        let result = controller.step(0.05 as f64, 0.01);

        assert_flexible!(result, ((3.0) + (6.0 - 0.05) * 0.01) * 0.5, 0.01)
    }

    #[test]
    //In this simulation the assumed process being controlled is velocity
    //The value is position
    fn output_simulation() -> std::io::Result<()> {
        const TARGET_MAXIMUM: f64 = 5.0; //Amplitude
        const SIMULATION_TIME: i64 = 100; //Seconds
        const SAMPLES_PER_SECOND: i64 = 50; //Hz
        const INCREMENT: f64 = 1.0 / SAMPLES_PER_SECOND as f64; //Seconds

        //All the external variables, like target and the plant function
        struct Parameters<T>
        where
            T: Floats,
        {
            target_maximum: T,
            plant_location: T,
            plant_velocity: T,
        }

        impl<T> Parameters<T>
        where
            T: Floats + From<f64>,
        {
            fn find_target(&self, time: T) -> T {
                if time > T::from(INCREMENT * 10.0) {
                    self.target_maximum
                } else {
                    T::from(0.0)
                }
            }

            fn plant_function(&mut self, input: T) -> T
            {
                self.plant_velocity += input * T::from(INCREMENT);
                self.plant_location += self.plant_velocity * T::from(INCREMENT);
                self.plant_location
            }
        }

        use std::fs::File;
        use std::io::Write;

        let mut f = File::create_new("PID_simulation.csv")?;

        let mut external = Parameters {
            target_maximum: TARGET_MAXIMUM,
            plant_location: 0.0,
            plant_velocity: 0.0,
        };

        let mut controller = Pid::new(0.8, 0.1, 0.01, 0.0, 0.0, 0.0);
        let mut measured: f64 = 0.0;

        f.write_all(
            "Time, Target, Measured, Previous Error, Cumulative Error, Output\n".as_bytes(),
        )?;

        for index in 1..SIMULATION_TIME * SAMPLES_PER_SECOND {
            let time = index as f64 * INCREMENT;
            controller.target = external.find_target(time);

            measured = external.plant_function(controller.step(measured, INCREMENT));

            let line = std::format!(
                "{}, {}, {}, {} ,{}\n",
                time,
                controller.target,
                measured,
                controller.previous_error,
                controller.cumulative_error
            );

            f.write_all(line.as_bytes())?;
        }

        Ok(())
    }
}
