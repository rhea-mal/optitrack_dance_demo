import matplotlib.pyplot as plt
import numpy as np
import time

# Initial values for the two categories
values1 = [0.5, 0.8]  # Data for "Hannah"
values2 = [100, 200]   # Data for "Tracy"

# Define the number of bars and their width
bar_width = 0.35
x = np.arange(len(values1))  # The label locations

plt.style.use('dark_background')

# Create a figure and two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.patch.set_facecolor('black')

# Adjust the x positions for the bars to reduce spacing
x1 = x - bar_width / 2  # Position for "Hannah"
x2 = x + bar_width / 2  # Position for "Tracy"

# Plot for "Hannah"
bars1 = ax1.bar(x1, values1, width=bar_width, label='Hannah', color='b')
ax1.set_ylabel('Kinetic Energy Ratio', color='white')
ax1.tick_params(axis='y', labelcolor='white')
ax1.set_yticks([0, 10, 20])
ax1.set_ylim(0, 20)
ax1.set_xticks(x)
# ax1.set_xticklabels(['Kinetic Energy Ratio', 'Power Ratio'])
ax1.set_xticklabels(['Hannah', 'Tracy'])
ax1.set_title('Kinetic Energy Ratio', color='white')

# Plot for "Tracy"
bars2 = ax2.bar(x2, values2, width=bar_width, label='Tracy', color='r')
ax2.set_ylabel('Power Ratio', color='white')
ax2.tick_params(axis='y', labelcolor='white')
ax2.set_yticks([0, 50, 100, 150, 200, 250])
ax2.set_ylim(0, 250)
ax2.set_xticks(x)
ax2.set_xticklabels(['Hannah', 'Tracy'])
ax2.set_title('Power Ratio', color='white')

# Combine legends
# ax1.legend(loc='upper left', facecolor='black', edgecolor='white')
# ax2.legend(loc='upper left', facecolor='black', edgecolor='white')

# Function to update the values
def update_values():
    global values1, values2
    # Generate values for "Hannah" scaled to the left y-axis (0-20)
    values1 = [np.random.rand() * 20 for _ in range(2)]
    # Generate values for "Tracy" scaled to the right y-axis (0-250)
    values2 = [np.random.randint(0, 250) for _ in range(2)]

# Iteratively update the values in a while loop
try:
    while True:
        update_values()

        # Update bar heights for "Hannah"
        for bar, new_height in zip(bars1, values1):
            bar.set_height(new_height)

        # Update bar heights for "Tracy"
        for bar, new_height in zip(bars2, values2):
            bar.set_height(new_height)

        # Redraw the canvas
        plt.draw()
        plt.pause(1)  # Pause for 1 second
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    plt.close(fig)  # Close the figure when done
