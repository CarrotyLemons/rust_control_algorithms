# Overview
Hello!
This is a rust library implementing a PID controller.
- Basic PID functioning

It includes some non-extensive tests and one output of a PID running process. This process involves the control of acceleration to try and reach a position.

The `PID_visualiser.py` can be run on the output file to show the PID display of the example case in the file.

# Future implementation
- Derivative filtering (imported from filtering crate)
    - Output diagrams for both with and without
- Clamping to prevent integral windup
    - Output diagrams for both with and without

# Usage
## PID
To use the PID struct, the preferred method is to create a struct with only the values you want filled in. Then allow the rest to be filled in by the `::blank` associated function.

This can be seen in the following example
```rs
let mut controller = Pid {
    kp: 0.2,
    target: 15.2,
    ..Pid::blank() //The default values are filled in
};
```

The `.step` method can then be called to iterate the pid with the current measured result

```rs
let result = controller.step(3.5 as f64, 0.01);

assert_eq!(result, 0.2 * (15.2 - 3.5))
```