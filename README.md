# Overview

This is a rust library implementing a PID controller. It has the following features
- Basic PID functioning
- Clamping as wind-up control

While I have tried to make the library code nice to work with, the tests have not had the same effort.

It includes some non-extensive tests:
    - Individual checks of the P, I and D functions
    - A non-testing output of a PID controller with clamping enabled, try disabling the clamping to see some wild instability (the output of the PID is constrained to show how clamping might be useful)

The `PID_visualiser.py` can be run on the output file to show the PID display of the example case.

# Future implementation
- Derivative filtering (imported from filtering crate)
    - Output diagrams for both with and without
- Allowing measurement of process variable for derivative term instead of error

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

## Clamping
Clamping is controlled by an Enum
```rs
pub enum Clamping<T> 
where
    T: Floats,
{
    None,
    LowerLimit(T),
    UpperLimit(T),
    BothLimits(T, T) //Upper, Lower
}

let mut controller = Pid {
    kp: 0.2,
    target: 15.2,
    clamping: Clamping::UpperLimit(12.0)
    ..Pid::blank() //The default values are filled in
};
```