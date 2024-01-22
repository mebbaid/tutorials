# showing the stability of sets

import numpy as np
from scipy.integrate import odeint
from math import log
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class DynamicSystem:
    def __init__(self, system_function):
        self.system_function = system_function

    def simulate(self, x0, t):
        assert x0.shape == (4,), "x0 must be a 4x1 vector"
        assert isinstance(t, np.ndarray), "t must be a numpy array"
        assert t.ndim == 1, "t must be a 1D array"

        return odeint(self.system_function, x0, t)

    def equilibrium_points(self):
        pass

    def animate(self, x0, t, filename=None):
        assert x0.shape == (4,), "x0 must be a 4x1 vector"
        assert isinstance(t, np.ndarray), "t must be a numpy array"
        assert t.ndim == 1, "t must be a 1D array"

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('$x_1$')
        ax.set_ylabel('$x_2$')
        ax.set_zlabel('$x_3$')
        ax.set_title('Dynamics of the system')

        x = self.simulate(x0, t)

        ax.plot(x[:, 0], x[:, 1], x[:, 2], 'b-')
        # The set to be analyzed is an ellptic paraboloid immersed in the subspace x4 = 0
        # i.e. {x in R^4 | x1^2 + x2^2 - x3^2 = x4 = 0}

        # plotting the set
        x1 = np.linspace(-5, 5, 100)
        x2 = np.linspace(-5, 5, 100)
        x1, x2 = np.meshgrid(x1, x2)
        x3 = x1**2 + x2**2
        ax.plot_surface(x1, x2, x3, alpha=0.5)


        if filename is not None:
            ani = animation.FuncAnimation(fig, self.animate_update, len(t),
                                          fargs=(x, ax), interval=1000 * t[-1] / len(t),
                                          blit=False)
            ani.save(filename, fps=30, extra_args=['-vcodec', 'libx264'])
        else:
            plt.show()

    def animate_update(self, frame, data, ax):
        ax.clear()
        ax.set_xlabel('$x_1$')
        ax.set_ylabel('$x_2$')
        ax.set_zlabel('$x_3$')
        ax.set_title('Dynamics of the system')

        # plotting the set
        x1 = np.linspace(-5, 5, 100)
        x2 = np.linspace(-5, 5, 100)
        x1, x2 = np.meshgrid(x1, x2)
        x3 =  x1**2 + x2**2
        ax.plot_surface(x1, x2, x3, alpha=0.5)

        ax.plot(data[:frame, 0], data[:frame, 1], data[:frame, 2], 'b-')
        x_eq = self.equilibrium_points()
        ax.plot

        # dynamic camera angle every frame
        ax.view_init(30, 30 + 360 * frame / len(data))

        return ax


# the example is a dynamics of 4D described by the following system
# x1' = -x2 - x2 * u2
# x2' = x1 + x1 * u2
# x3' = x3*x4 + x3*u1
# x4' = u1
# where u1 and u2 are controls computed by a controller function with the following law
# z = [x(1), x(2), log(x(3)/(x(1)^2+x(2)^2)) - x(4), x(4)]
# u1 = -2*z(3)-2*z(4);
# u2 = 0.0

# the system function

def system_function(x, t):
    assert x.shape == (4,), "x must be a 4x1 vector"
    assert isinstance(t, float), "t must be a float"

    u1 = -2* (log(x[2]/(x[0]**2 + x[1]**2)) - x[3]) - 2*x[3]
    u2 = 0.0

    return np.array([-x[1] - x[1] * u2,
                     x[0] + x[0] * u2,
                     x[2]*x[3] + x[2]*u1,
                     u1])



# testing the system
t = np.linspace(0, 15, 1500)
x0 = np.array([5, 2, 2, 0.5])
system = DynamicSystem(system_function)
system.animate(x0, t, filename='stability_of_sets_1.mp4')

x0 = np.array([-4, 0, 2, 0.5])
system = DynamicSystem(system_function)
system.animate(x0, t, filename='stability_of_sets_2.mp4')