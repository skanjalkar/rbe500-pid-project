import numpy as np
import matplotlib.pyplot as plt

def visualization():
    fig, ax = plt.subplots(2)
    fig.suptitle("Joint Values over Time")

    ax[0].grid(visible=True)
    ax[1].grid(visible=True)

    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    time = np.array(range(trajectory.shape[0])).reshape(trajectory.shape[0], 1)
    trajectory = np.hstack((time, trajectory))

    ax[0].plot(trajectory[:, 0], trajectory[:, 1], linewidth=2, label='Joint1')
    ax[0].plot(trajectory[:, 0], trajectory[:, 2], linewidth=2, label='Joint2')
    ax[1].plot(trajectory[:, 0], trajectory[:, 3], linewidth=2, label='Joint3')

    ax[0].set_title('Joint1 & Joint2 Values')
    ax[1].set_title('Joint3 Values')
    ax[0].set_ylabel("Radians")
    ax[1].set_ylabel("Meters")
    ax[1].set_xlabel("Time")
    ax[0].legend()
    ax[1].legend()


    plt.show()

if __name__ == '__main__':
    visualization()