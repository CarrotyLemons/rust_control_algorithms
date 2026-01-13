mod tests {
    use control_algorithms::{Pid, Clamping};
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
            previous_measurement: -0.1,
            ..Pid::blank()
        };
        let result = controller.step(0.05 as f64, 0.01);

        assert_eq!(result, 2.2 * (0.05 -- 0.1) / (0.01))
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
        let result = controller.step(0.05_f64, 0.01_f64);

        assert_eq!(result, ((3.0_f64) + (6.0_f64 - 0.05_f64) * 0.01_f64) * 0.5_f64)
    }

    #[test]
    // In this simulation the assumed process being controlled is velocity
    // The value is position
    fn output_simulation() -> std::io::Result<()> {
        const SIMULATION_TIME: f64 = 100.0;
        const SIMULATION_STEP: f64 = 0.01;

        struct Parameters
        {
            time_delayed_output: Vec<f64>
        }

        impl Parameters
        {
            fn calculate_target(&self, time: f64) -> f64 {
                if time > 10.0 {
                    75.0
                } else {
                    0.0
                }
            }

            fn plant_function(&mut self, input: f64) -> f64
            {
                self.time_delayed_output.push(input);
                self.time_delayed_output.pop().unwrap()
            }
        }

        use std::fs::File;
        use std::io::Write;

        let mut f = File::create("tests/PID_simulation.csv")?;

        let mut external = Parameters {
            time_delayed_output: Vec::with_capacity(10),
        };

        let mut controller = Pid {
            kp: 0.1,
            kd: 0.0,
            ki: 0.05,
            // clamping: Clamping::BothLimits(-15.0, 15.0),
            clamping: Clamping::None,
            ..Pid::blank()
        };
        let mut measured: f64 = 0.0;

        f.write_all(
            "Time, Target, Measured, Previous Error, Cumulative Error, Output, Clamping\n".as_bytes(),
        )?;

        for index in 1..(SIMULATION_TIME / SIMULATION_STEP) as i64 {
            let time = index as f64 * SIMULATION_STEP;
            controller.target = external.calculate_target(time);

            measured = external.plant_function(controller.step(measured, SIMULATION_STEP));

            let line = std::format!(
                "{}, {}, {}, {}, {}\n",
                time,
                controller.target,
                measured,
                controller.cumulative_error,
                controller.clamping.exceeded(controller.cumulative_error)
            );

            f.write_all(line.as_bytes())?;
        }

        Ok(())
    }
}