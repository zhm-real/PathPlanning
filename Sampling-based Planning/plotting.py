import matplotlib.pyplot as plt
import matplotlib.patches as patches
import env


class Plotting:
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs

    def animation(self, nodelist, path):
        if path is None:
            print("No path found!")
            return

        self.plot_visited(nodelist)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        for x in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (x[0], x[1]), x[2], x[3],
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for x in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (x[0], x[1]), x[2],
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)
        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist):
        for node in nodelist:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                plt.pause(0.001)

    @staticmethod
    def plot_path(path):
        plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
        plt.pause(0.01)
        plt.show()
