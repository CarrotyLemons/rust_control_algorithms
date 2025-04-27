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

plt.plot(time, measured, label = "Measured")
plt.plot(time, target, label = "Target", linestyle = ':')
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("PID rust simulation output")
plt.legend()
plt.grid()
plt.show()