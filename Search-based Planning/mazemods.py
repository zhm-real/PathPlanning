#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Huiming Zhou
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


# Makes the maze data with obstacles stored as black RGB values,
# free space stored as white RGB values"
def makeMaze(n, m, O):
    # Initialize to lists of 1. for RGB color index = white
    gridvals = [[[1. for i in range(3)] for col in range(n)] for row in range(m)]
    # Iterate through each obstacle
    for l in range(len(O)):
        # Find boundaries of current obstacle
        west, east = [O[l][0], O[l][1]]
        south, north = [O[l][2], O[l][3]]
        # Iterate through each cell of obstacle (clunky, but works)
        for i in range(west, east + 1):
            for j in range(south, north + 1):
                gridvals[j][i] = [0., 0., 0.]  # Change entry to RGB black
    return gridvals


# Function to actually plot the maze
def maze(n, m, O):
    gridvals = makeMaze(n, m, O)
    fig, ax = plt.subplots()  # make a figure + axes
    ax.imshow(gridvals)  # Plot it
    ax.invert_yaxis()  # Needed so that bottom left is (0,0)
    # ax.axis('off')


# Checks for collisions given position x, control u, obstacle list O
def collisionCheck(x, u, O):
    # Check input
    if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1):
        print('collision_check error: Invalid input u!')
        return
    nextx = [x[i] + u[i] for i in range(len(x))]
    for l in range(len(O)):
        # Find boundaries of current obstacle
        west, east = [O[l][0], O[l][1]]
        south, north = [O[l][2], O[l][3]]
        # Check if nextx is contained in obstacle boundaries
        if west <= nextx[0] <= east and south <= nextx[1] <= north:
            return True
    # If we iterate through whole list and don't trigger the "if", then no collisions
    return False


# Makes a piece of data with obstacles stored as black RGB values,
# free space stored as white RGB values, and path stored as increasing hue of
# yellow RGB values
def makePath(xI, xG, path, n, m, O):
    # Obtain the grid populated with obstacles and free space RGB values first
    gridpath = makeMaze(n, m, O)
    L = len(path)
    # Iterate through the path to plot as increasing shades of yellow
    for l in range(L - 1):
        gridpath[path[l][1]][path[l][0]] = [1., 1., 1 - l / (L - 1)]  # white-->yellow
    gridpath[xI[1]][xI[0]] = [0., 0., 1.]  # Initial node (plotted as blue)
    gridpath[xG[1]][xG[0]] = [0., 1., 0.]  # Goal node (plotted as green)
    return gridpath


# Constructs path list from initial point and list of actions
def getPathFromActions(xI, actions):
    L = len(actions)
    path = []
    nextx = xI
    for l in range(L):
        u = actions[l]
        if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1):
            print('getPath error: Invalid input u!')
            return
        nextx = [nextx[i] + u[i] for i in range(len(nextx))]  # nextx = nextx + u
        path.append(nextx)  # Builds the path
    return path


# If any collisions, cost is 999999, else cost is one for each action
def getCostOfActions(xI, actions, O):
    L = len(actions)
    costsum = 0
    nextx = xI
    for l in range(L):
        u = actions[l]
        if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1):
            print('getCostOfActions error: Invalid input u!')
            return
        collision = collisionCheck(nextx, u, O)
        if collision: return 999999999
        nextx = [nextx[i] + u[i] for i in range(len(nextx))]  # nextx = nextx + u
        costsum = costsum + 1
    return costsum


# If any collisions, cost is 999999, else cost is 2^(x[0]) for each action
def stayWestCost(xI, actions, O):
    L = len(actions)
    costsum = 0
    nextx = xI
    for l in range(L):
        u = actions[l]
        if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1):
            print('stayWestCost error: Invalid input u!')
            return
        collision = collisionCheck(nextx, u, O)
        if collision: return 999999999
        nextx = [nextx[i] + u[i] for i in range(len(nextx))]  # nextx = nextx + u
        costsum = costsum + nextx[0] ** 2
    return costsum


# If any collisions, cost is 999999, else cost is 2^(maxX - x[0]) for each action
def stayEastCost(xI, actions, O):
    # Determine maximum x coordinate of workspace from obstacle list
    maxX = 0
    for k in range(len(O)):
        westxO = O[k][1]
        if westxO > maxX:
            maxX = westxO

    L = len(actions)
    costsum = 0
    nextx = xI
    for l in range(L):
        u = actions[l]
        if u != (-1, 0) and u != (1, 0) and u != (0, -1) and u != (0, 1):
            print('stayEastCost error: Invalid input u!')
            return
        collision = collisionCheck(nextx, u, O)
        if collision: return 999999999
        nextx = [nextx[i] + u[i] for i in range(len(nextx))]  # nextx = nextx + u
        costsum = costsum + (maxX - nextx[0]) ** 2
    return costsum


# Calculate different types of cost for searching algorithm
def cost_calculation(xI, actions, O):
    simple_cost = getCostOfActions(xI, actions, O)
    west_cost = stayWestCost(xI, actions, O)
    east_cost = stayEastCost(xI, actions, O)
    return simple_cost, west_cost, east_cost


# Extract path from results of searching algorithm
def extractpath(xI, xG, parent, actions):
    pathback = [xG]
    actionsback = []
    actionsback.append(actions[xG])
    x_current = xG
    while parent[x_current] != x_current:
        x_current = parent[x_current]
        pathback.append(x_current)
        actionsback.append(actions[x_current])
    path_extract = list(reversed(pathback))
    actions_extract = list(reversed(actionsback))
    path_extract.pop(0)
    actions_extract.pop(0)
    return path_extract, actions_extract


# Plots the path
def showPath(xI, xG, path, n, m, O, name):
    gridpath = makePath(xI, xG, path, n, m, O)
    fig, ax = plt.subplots(1, 1)  # make a figure + axes
    ax.imshow(gridpath)  # Plot it
    ax.invert_yaxis()  # Needed so that bottom left is (0,0)
    plt.title(name, fontdict=None)
    plt.show()



