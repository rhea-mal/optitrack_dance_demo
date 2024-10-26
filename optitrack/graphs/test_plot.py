import redis
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Connect to Redis
r = redis.Redis(host='localhost', port=6379, db=0)

# Define the keys to fetch data for each bar chart
HANNAH_ROBOT_KINETIC_KEY = "sai2::sim::hannah::robot::kinetic";
TRACY_ROBOT_KINETIC_KEY = "sai2::sim::tracy::robot::kinetic";
HANNAH_HUMAN_KINETIC_KEY = "sai2::sim::hannah::human::kinetic";
TRACY_HUMAN_KINETIC_KEY = "sai2::sim::tracy::human::kinetic";

HANNAH_ROBOT_EFFORT_KEY = "sai2::sim::hannah::robot::effort";
TRACY_ROBOT_EFFORT_KEY = "sai2::sim::tracy::robot::effort";
HANNAH_HUMAN_EFFORT_KEY = "sai2::sim::hannah::human::effort";
TRACY_HUMAN_EFFORT_KEY = "sai2::sim::tracy::human::effort";

# Initial data (use placeholder values initially)
data1 = [0, 0]  # hannah, tracy kinetic ratio 
data2 = [0, 0]  # hannah, tracy effort ratio
labels = ['Hannah', 'Tracy']

# Create figure and subplots
plt.style.use('dark_background')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 10))
bar1 = ax1.bar(labels, data1, color=['blue', 'red'])
bar2 = ax2.bar(labels, data2, color=['blue', 'red'])
ax1.set_title('Kinetic Energy Ratio (Robot : Human)')
# ax1.set_ylabel('Values')
ax2.set_title('Power Ratio (Robot : Human)')
# ax2.set_ylabel('Values')

# Set custom y-ticks for each subplot
ax1.set_yticks([0, 1, 2])  # Left subplot y-ticks
ax2.set_yticks([0, 5, 10])         # Right subplot y-ticks

# Function to update the bars in the plot
def update(frame):
    global data1, data2
    try:
        # Fetch data from Redis
        hannah_robot_kinetic = float(r.get(HANNAH_ROBOT_KINETIC_KEY))
        tracy_robot_kinetic = float(r.get(TRACY_ROBOT_KINETIC_KEY))
        hannah_human_kinetic = float(r.get(HANNAH_HUMAN_KINETIC_KEY))
        tracy_human_kinetic = float(r.get(TRACY_HUMAN_KINETIC_KEY))

        hannah_robot_effort = float(r.get(HANNAH_ROBOT_EFFORT_KEY))
        tracy_robot_effort = float(r.get(TRACY_ROBOT_EFFORT_KEY))
        hannah_human_effort = float(r.get(HANNAH_HUMAN_EFFORT_KEY))
        tracy_human_effort = float(r.get(TRACY_HUMAN_EFFORT_KEY))

        tol = 1e0

        if abs(hannah_human_kinetic) < tol:
            data1[0] = 1
        else:
            data1[0] = hannah_robot_kinetic / hannah_human_kinetic 

        if abs(tracy_human_kinetic) < tol:
            data1[1] = 1
        else:
            data1[1] = tracy_robot_kinetic / tracy_human_kinetic

        if abs(hannah_human_effort) < tol:
            data2[0] = 1
        else:
            data2[0] = hannah_robot_effort / hannah_human_effort

        if abs(tracy_human_effort) < tol:
            data2[1] = 1
        else:
            data2[1] = tracy_robot_effort / tracy_human_effort
        
        # Update the first subplot bars
        for rect, h in zip(bar1, data1):
            rect.set_height(h)

        # Update the second subplot bars
        for rect, h in zip(bar2, data2):
            rect.set_height(h)

    except Exception as e:
        print(f"Error updating data: {e}")

# Animate the plot with a 1-second interval
ani = FuncAnimation(fig, update, interval=10)

plt.tight_layout()
plt.show()
