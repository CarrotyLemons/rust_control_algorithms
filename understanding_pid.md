# Overview
This is just a spot for documentation on how I understand the PID control algorithm.

It won't completely explain them, just aspects I would like to remember.

It is also my notes, so if something sounds incorrect please let me know :)

# General
Designed to operate on systems with an unknown function transfering the output to the state of the object.

If you attempt to apply PID controllers to something direct (like a plant function that is a 1-to-1 function of the output) that results in strange behaviour.

PID controllers make sense when applied to something like the acceleration of an object, where you want to control position.

They rely on calculating the error relevant to their goal and creating different terms (proportional, integral and derivative) to approach the solution in different ways.

# Anti-windup scheme