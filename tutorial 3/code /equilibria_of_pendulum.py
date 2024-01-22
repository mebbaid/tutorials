import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Physical parameters
g = 9.81
l = 1.0
m = 1.0
d = 0.1
b = d / (m * l**2)

# Pendulum dynamics function limiting to (-pi, pi)
def pendulum(x, t):
    return np.array([x[1], -b * x[1] - g / l * np.sin(x[0])])

# Simulation function
def simulate_pendulum(x0, t):
    return odeint(pendulum, x0, t)

# Animation update function
def update_animation(frame, data, pendulum_line, pendulum_rod, ax):
    ax.clear()
    ax.set_xlabel('$x_1$')
    ax.set_ylabel('$x_2$')
    ax.set_title('Dynamics of the pendulum')

    # Plotting the pendulum trajectory
    ax.plot(data[:frame, 0], data[:frame, 1], 'b-')

    # Plotting the two equilibrium points
    ax.plot([0], [0], 'go')
    ax.plot([np.pi], [0], 'ro')


    # Updating the pendulum line
    pendulum_line.set_data(data[:frame, 0], data[:frame, 1])

    # Updating the pendulum rod
    pendulum_rod.set_data([0, np.sin(data[frame, 0])], [-l, -l - np.cos(data[frame, 0])])

    ax.legend(['Trajectory', 'Stable equilibrium', 'Unstable equilibrium'])

    return pendulum_line, pendulum_rod



# Animation function
def animate_pendulum(x0, t, filename=None):
    fig, ax = plt.subplots()
    ax.set_xlabel('$x_1$')
    ax.set_ylabel('$x_2$')
    ax.set_title('Trajectories of the pendulum')

    x = simulate_pendulum(x0, t)

    pendulum_line, = ax.plot(x[0, 0], x[0, 1], 'b-')
    ax.plot([0], [0], 'go')
    ax.plot([np.pi], [0], 'ro')

    # add legend
    ax.legend(['Trajectory', 'Stable equilibrium', 'Unstable equilibrium'])
    pendulum_rod, = ax.plot([], [], 'k-', lw=2)

    ani = FuncAnimation(fig, update_animation, len(t),
                        fargs=(x, pendulum_line, pendulum_rod, ax),
                        interval=1000 * t[-1] / len(t), blit=True)

    if filename is not None:
        ani.save(filename, fps=30, extra_args=['-vcodec', 'libx264'])
    else:
        plt.show()

# Initial condition
initial_condition = np.array([np.pi / 6, 0])

# Time interval to simulate
time_array = np.linspace(0, 20, 2000)

# Animate the pendulum
animate_pendulum(initial_condition, time_array, filename='pendulum.mp4')


# Initial condition
initial_condition = np.array([2* np.pi / 3, 0])

# Time interval to simulate
time_array = np.linspace(0, 20, 2000)

# Animate the pendulum
animate_pendulum(initial_condition, time_array, filename='pendulum_2.mp4')

# Initial condition
initial_condition = np.array([np.pi  + 0.05, 0])

# Time interval to simulate
time_array = np.linspace(0, 20, 2000)

# Animate the pendulum
animate_pendulum(initial_condition, time_array, filename='pendulum_3.mp4')