import matplotlib.pyplot as plt
import numpy as np
import time, math

# Generates a scatter plot of function run times against the size of the map.
class PlotTimer:
    def __init__(self, node):
        self.node = node
        self.map = node.map
        self.seconds = []
        self.pixels = []
        self.startTime = 0
    
    def start(self):
        self.startTime = time.time()
    
    def stop(self):
        end = time.time()
        self.seconds.append(end - self.startTime)
        self.pixels.append(self.node.map.info.width * self.node.map.info.height)
    
    def plot(self, color='r', title='Runtime', yscale=0.05):
        plt.clf()
        plt.scatter(
            self.pixels, 
            self.seconds,
            linestyle='--', marker='o', color=color
        )
        plt.yticks(np.arange(math.floor(min(self.seconds)), math.ceil(max(self.seconds)), yscale))
        plt.ylabel('Seconds')
        plt.xlabel('Map Pixels')
        plt.plot(np.unique(self.pixels), np.poly1d(np.polyfit(self.pixels, self.seconds, 1))(np.unique(self.pixels)))
        plt.title(title)
        plt.pause(0.05)