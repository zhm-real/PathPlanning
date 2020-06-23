import matplotlib.pyplot as plt
import matplotlib.patches as patches
import env


class Plotting:
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle

    def animation(self, nodelist, node_rand=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if node_rand is not None:
            plt.plot(node_rand.x, node_rand.y, "^k")

        for node in nodelist:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        plt.plot(self.xI[0], self.xI[1], "b*")
        plt.plot(self.xG[0], self.xG[1], "g*")

        plt.axis("equal")
        plt.axis([-5, 55, -5, 35])
        plt.title("RRT")
        plt.grid(True)
        plt.pause(0.01)

    def plot_grid(self, name):
        fig, ax = plt.subplots()
        for x in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (x[0], x[1]), x[2], x[2],
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for x in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (x[0], x[1]), x[2],
                    edgecolor='gray',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.xI[0], self.xI[1], "b*")
        plt.plot(self.xG[0], self.xG[1], "g*")

        plt.title(name)
        plt.axis("equal")
        plt.show()

    def plot_visited(self, visited):
        visited.remove(self.xI)
        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], linewidth='3', color='#808080', marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 15
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 45

            if count % length == 0: plt.pause(0.001)

    def plot_path(self, path):
        path.remove(self.xI)
        path.remove(self.xG)
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]

        plt.plot(path_x, path_y, linewidth='3', color='r', marker='o')
        plt.pause(0.01)
        plt.show()
