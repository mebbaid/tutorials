import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

# Define the linear system
def linear_system(x, t, A):
    return A.dot(x)

# Simulate the system
def simulate(x0, t, A):
    return odeint(linear_system, x0, t, args=(A,))

# Define the nullspace of A
def nullspace(A, atol=1e-13, rtol=0):
    u, s, vh = np.linalg.svd(A)
    tol = max(atol, rtol * s[0])
    nnz = (s >= tol).sum()
    ns = vh[nnz:].conj().T
    # Ensure ns has three columns
    if ns.shape[1] == 1:
        ns = np.hstack((ns, np.zeros((len(ns), 2))))
    return ns

# Compute the equilibrium points of a system
def equilibrium_points(A):
    return nullspace(A)

# Animate the system in the 3D state space
def animate_update(frame, data, line, nullspace_line, eq_points, ax):
    line.set_data(data[:frame, 0], data[:frame, 1])
    line.set_3d_properties(data[:frame, 2])

    # Update nullspace line
    nullspace_line.set_data([0, eq_points[0, 0]], [0, eq_points[1, 0]])
    nullspace_line.set_3d_properties([0, eq_points[2, 0]])

    return line, nullspace_line

def animate_system(data, eq_points, ns, filename='linear_system.mp4'):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('$x_1$')
    ax.set_ylabel('$x_2$')
    ax.set_zlabel('$x_3$')
    ax.set_title('Dynamics of the system')

    # Plot initial condition
    ax.plot([data[0, 0]], [data[0, 1]], [data[0, 2]], 'go')

    # Plot the nullspace line
    nullspace_line, = ax.plot([], [], [], 'r-', lw=2)

    # Plot equilibrium points
    ax.scatter(eq_points[0, :], eq_points[1, :], eq_points[2, :], c='orange', marker='o', label='Equilibrium Points')

    # Auto scale ax limits to fit data, nullspace line, and equilibrium points
    ax.set_xlim(np.min(data[:, 0]) - 0.1, np.max(data[:, 0]) + 0.1)
    ax.set_ylim(np.min(data[:, 1]) - 0.1, np.max(data[:, 1]) + 0.1)
    ax.set_zlim(np.min(data[:, 2]) - 0.1, np.max(data[:, 2]) + 0.1)

    ax.set_aspect('auto')
    ax.view_init(30, 30)

    line, = ax.plot([], [], [], 'b-', lw=2)
    ax.legend(['Trajectory', 'Nullspace Line', 'Equilibrium Points'])

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        return line, nullspace_line

    ani = animation.FuncAnimation(fig, animate_update, len(data),
                                  fargs=(data, line, nullspace_line, eq_points, ax),
                                  init_func=init, blit=False)
    ani.save(filename, fps=30, extra_args=['-vcodec', 'libx264'])

# Testing different initial conditions
initial_conditions = [
    np.array([0, 0.5, 1]),
    np.array([0.5, 0, 0.5])
]

A = np.array([[-1, 0, 1], [0, -2, 0], [1, 1, -1]])

eq_points = equilibrium_points(A)

for i, x0 in enumerate(initial_conditions):
    t = np.linspace(0, 5, 1000)
    x = simulate(x0, t, A)
    ns = nullspace(A)

    animate_system(x, eq_points, ns, filename=f'linear_system_{i}.mp4')
