# Overview
This is just a spot for documentation on how I understand the PID control algorithm.

It won't completely explain them, just aspects I would like to remember.

It is also my notes, so if something sounds incorrect please let me know :)

# General
Designed to operate on systems with an unknown function transfering the output to the state of the object.

If you attempt to apply PID controllers to something direct (like a plant function that is a 1-to-1 function of the output) that results in strange behaviour.

PID controllers make sense when applied to something like the acceleration of an object, where you want to control position.

They rely on calculating the error relevant to their goal and creating different terms (proportional, integral and derivative) to approach the solution in different ways.

# Anti-windup (the integral term)
When the plant process has a time delay, or there are sharp changes in the goal of the control algorithm. Something called *integral windup* can happen where the integral component of the controller grows very large.

This can be mitigated by using anti-windup techniques such as clamping, back-calculation and having setpoint ramping to smooth the setpoint change (not effective in the case of a time delayed plant process).

Windup occurs when the output of the PID controller exceeds the bounds the plant is capable of producing, *saturating* the output. One solution to this implemented here is clamping, which entirely disables the accumulation of error and the output of the integral term until the output of the PID controller return back into the set bounds.

# Filtering (the derivative term)
Filtering the input to the derivative term of the PID controller can help remove a lot of instability if the PID controller is receiving noisy data. 

The derivative term can also cause unstable behaviour when dealing with sudden changes associated with setpoint. Setpoint ramping can resolve this as well. This can be mitigated by instead taking the derivative of the measured process variable instead of the error.