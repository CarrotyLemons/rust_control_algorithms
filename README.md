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