"""Used to visualise the output of the tests when run, and graph them."""

# pyright: reportUnknownMemberType = false

import csv

import matplotlib.pyplot as plt

csv_path = "PID_simulation.csv"

with open(csv_path) as file:
    reader = csv.reader(file)
    next(reader)
    data = list(reader)

time = [float(row[0]) for row in data]
target = [float(row[1]) for row in data]
measured = [float(row[2]) for row in data]
cumulative_error = [float(row[3]) for row in data]
clamping_exceeded = [bool(row[4]) for row in data]

plt.plot(time, measured, label = "Measured")
plt.plot(time, target, label = "Target", linestyle = ":")
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("PID simulation output")
plt.legend()
plt.grid()
plt.savefig("visualiser_output.png")
plt.show()
