import pygame
from RRTBase import RRTGraph, RRTMap


def main():
    dimensions = (600, 600)

    start = (50, 50)

    goal = (550, 550)

    obsdim = 30

    obsnum = 50

    pygame.init()

    map = RRTMap(start, goal, dimensions, obsdim, obsnum)

    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()

    map.drawMap(obstacles)

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    main()
