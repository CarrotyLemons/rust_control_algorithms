mod tests {
    use control_algorithms::{Pid, Clamping};
    use rust_helper_tools::{assert_flexible, Floats};
    extern crate std;

    #[test]
    // Test the use of an exclusively proportional PID
    fn test_proportional_pid() {
        let mut controller = Pid {
            kp: 0.2,
            target: 15.2,
            ..Pid::blank()
        };
        let result = controller.step(3.5 as f64, 0.01);

        assert_eq!(result, 0.2 * (15.2 - 3.5))
    }

    #[test]
    // Test the use of an exclusively derivative PID
    fn test_derivative_pid() {
        let mut controller = Pid {
            kd: 2.2,
            target: 12.0,
            previous_error: -0.1,
            ..Pid::blank()
        };
        let result = controller.step(0.05 as f64, 0.01);

        assert_flexible!(result, 2.2 * ((12.0 - 0.05) + 0.1) / (0.01), 0.01)
    }

    #[test]
    // Test the use of an exclusively integral PID
    fn test_integral_pid() {
        let mut controller = Pid {
            ki: 0.5,
            target: 6.0,
            cumulative_error: 3.0,
            ..Pid::blank()
        };
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
        const PLANT_LOWER_LIMIT: f64 = -1.5;
        const PLANT_UPPER_LIMIT: f64 = 1.5;

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

            fn plant_function(&mut self, mut input: T) -> T
            {
                if input < T::from(PLANT_LOWER_LIMIT) {
                    input = T::from(-1.5);
                } else if input > T::from(PLANT_UPPER_LIMIT) {
                    input = T::from(1.5);
                }
                self.plant_velocity += input * T::from(INCREMENT);
                self.plant_location += self.plant_velocity * T::from(INCREMENT);
                self.plant_location
            }
        }

        use std::fs::File;
        use std::io::Write;

        let mut f = File::create("PID_simulation.csv")?;

        let mut external = Parameters {
            target_maximum: TARGET_MAXIMUM,
            plant_location: 0.0,
            plant_velocity: 0.0,
        };

        let mut controller = Pid {
            kp: 0.8,
            kd: 0.1,
            ki: 0.05,
            clamping: Clamping::BothLimits(-1.5, 1.5),
            ..Pid::blank()
        };
        let mut measured: f64 = 0.0;

        f.write_all(
            "Time, Target, Measured, Previous Error, Cumulative Error, Output, Clamping\n".as_bytes(),
        )?;

        for index in 1..SIMULATION_TIME * SAMPLES_PER_SECOND {
            let time = index as f64 * INCREMENT;
            controller.target = external.find_target(time);

            measured = external.plant_function(controller.step(measured, INCREMENT));

            let line = std::format!(
                "{}, {}, {}, {} ,{}, {}, {}\n",
                time,
                controller.target,
                measured,
                controller.previous_error,
                controller.cumulative_error,
                controller.output,
                controller.clamping.exceeded(controller.output)
            );

            f.write_all(line.as_bytes())?;
        }

        Ok(())
    }
}