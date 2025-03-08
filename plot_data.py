import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

# quadratic 
data = {
    0.5: [0.58, 0.63, 0.61, 0.69, 0.65, 0.69, 0.66, 0.66, 0.45, 0.42, 0.43],
    0.6: [0.72, 0.78, 0.73, 0.65, 0.62, 0.78, 0.58, 0.79],
    0.7: [0.70, 0.77, 0.77, 0.78, 0.75, 0.78, 0.82, 0.86, 0.57, 0.62, 0.72, 0.74, 0.9],
    0.8: [0.99, 0.89, 1.0, 0.85, 0.75, 0.88, 0.68],
    0.9: [0.89, 0.91, 1.44, 1.02, 0.94, 1.07, 0.99, 1.03, 0.81, 0.94],
    1.0: [0.88, 0.96, 1.07, 0.98, 1.00, 1.09, 1.01, 1.08, 1.15, 1.17, 1.19, 1.18, 1.25],
    1.2: [1.27, 1.27, 1.31, 1.26, 1.32, 1.19, 1.33, 1.4, 1.45],
    1.4: [1.55, 1.37, 1.49, 1.49, 1.57, 1.66, 1.42, 1.62, 1.65],
    1.6: [1.57, 1.55, 1.72, 1.62, 1.52, 1.7, 1.75, 1.68, 1.48],
    1.8: [1.64, 1.83, 1.79, 1.78, 1.81, 1.81, 1.62, 1.59, 1.95, 2.05],
    2.0: [2.22, 2.16, 1.89, 2.13, 2.14, 2.1, 2.1, 2.07, 2.03, 1.9],
    2.2: [2.14, 2.04, 2.26, 2.28, 2.05, 2.05, 1.9, 2.63, 2.3, 2.3],
    2.4: [2.36, 2.35, 2.28, 2.1, 2.31, 2.2, 2.33, 2.32, 2.28, 2.26],
    2.6: [2.37, 2.2, 2.25, 2.3, 2.19, 2.1, 2.47, 2.86],
    2.8: [2.2, 2.46, 2.37, 2.69, 2.3, 2.27, 2.3, 2.24, 2.15],
    3.0: [2.91, 2.8, 2.6, 2.5, 2.55, 2.46, 2.38, 2.71, 2.27]
}

# linear
data2 = {
    0.5: [0.366, 0.365, 0.38, 0.34, 0.359, 0.3, 0.3, 0.32],
    0.6: [0.539, 0.63, 0.589, 0.57],
    0.7: [0.85, 0.67, 0.85, 0.95, 0.68],
    0.8: [0.88, 1.06, 0.95, 1.0, 0.99, 0.95],
    0.9: [0.83, 1.02, 1.05],
    1.0: [0.78, 0.82, 0.92, 1.15, 1.2, 1.15, 1.18],
    1.2: [1.22, 1.05, 1.1, 1.42, 1.47],
    1.4: [1.45, 1.85, 1.7, 1.69, 1.97],
    1.6: [1.76, 1.92, 1.8, 1.75, 1.78],
    1.8: [1.92, 2.03, 2.0, 2.02, 1.97, 1.98, 1.709],
    2.0: [1.82, 1.98, 2.0, 1.88, 1.92, 2.02, 2.03, 2.6, 2.57],
    2.2: [2.15, 2.08, 2.0, 2.05],
    2.4: [2.25, 2.2, 2.3, 2.1, 2.0],
    2.6: [1.95, 2.02, 2.1, 2.25, 2.15],
    2.8: [2.4, 2.5, 2.52, 2.48, 2.45, 2.18],
    3.0: [2.27, 2.35, 2.55, 2.81]
}

goals = []
means = []
std_devs = []

for goal, measurements in data.items():
    goals.append(goal)
    means.append(np.mean(measurements))
    std_devs.append(np.std(measurements))

plt.figure(figsize=(10, 6))
plt.plot(goals, means, marker='o', label='Mean Position')
plt.fill_between(goals, np.array(means) - np.array(std_devs), 
                 np.array(means) + np.array(std_devs), color='b', alpha=0.2, label='Std Dev')

plt.plot(goals, goals, linestyle='--', color='r', label='Goal Line')

plt.xlabel('Goal Position (m)')
plt.ylabel('Measured Position (m)')
plt.title('Quadratic interpolation')
plt.legend()
plt.grid(True)
plt.show()

# Calculate average deviation of whole data (Mean Absolute Deviation - MAD)

def calculate_average_deviation(data):
    deviations = []

    for key, values in data.items():
        values = np.array(values)
        if key < 1.0 or key >= 2.0:
            continue
        deviations.append(np.abs(np.mean(values) - key))
    print(deviations)
    average_deviation = np.mean(deviations)
    
    return average_deviation

# Calculate for data1 (Quadratic Interpolation)
average_deviation_quadratic = calculate_average_deviation(data)

print(f"Average Deviation (Quadratic Interpolation): {average_deviation_quadratic:.3f}")
errors = {key: np.abs(np.array(values) - key) for key, values in data.items()}
ranges = [(0.5, 1.0), (1.0, 2.0), (2.0, 3.0)]
for r in ranges:
    filtered_errors = [err for key, values in errors.items() if r[0] <= key < r[1] for err in values]
    mean_error = np.mean(filtered_errors)
    std_error = np.std(filtered_errors)
    
    print(mean_error, std_error)

