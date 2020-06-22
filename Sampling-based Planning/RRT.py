import env
import plotting
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class RRT:
    def __init__(self, xI, xG):
        # Plotting = plotting.Plotting(xI, xG)
        # Plotting.animation([xI, xG], [xI, xG], "zhou")
        fig, ax = plt.subplots()

        plt.axis([-5, 5, -5, 5])

        ax.plot()

        ax.add_patch(
            patches.Rectangle(
                (1, 1),
                0.5,
                0.5,
                edgecolor='black',
                facecolor='black',
                fill=True
            ))

        ax.add_patch(
            patches.Circle(
                (3, 3),
                0.5,
                edgecolor='black',
                facecolor='black',
                fill=True
            )
        )

        plt.axis("equal")
        plt.show()


    def planning(self):
        return


if __name__ == '__main__':
    x_Start = (5, 5)  # Starting node
    x_Goal = (49, 5)  # Goal node

    rrt = RRT(x_Start, x_Goal)
