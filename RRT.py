import pygame
from RRTBase import RRTGraph, RRTMap


def main():
    mapDimensions = (600, 600)
    startPosition = (50, 50)
    goalPosition = (550, 550)

    obstacleSize = 30
    obstacleNumber = 50

    pygame.init()

    map = RRTMap(
        startPosition, goalPosition, mapDimensions, obstacleSize, obstacleNumber
    )
    graph = RRTGraph(
        startPosition, goalPosition, mapDimensions, obstacleSize, obstacleNumber
    )

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    count = 0
    range = 15
    while count < range:

        count += 1
        x, y = graph.generate_random_sample()

        nodesNumber = graph.number_of_nodes()
        graph.add_node(nodesNumber, x, y)
        graph.add_edge(nodesNumber - 1, nodesNumber)

        # If has not collision, draw
        if graph.isFree():
            print("isFree")
            pygame.draw.circle(
                map.map,
                map.MapSettings["colors"]["path"],
                (graph.x[nodesNumber], graph.y[nodesNumber]),
                map.nodeRad,
                map.nodeThickness,
            )

            currentNodePos = graph.x[nodesNumber], graph.y[nodesNumber]
            lastNodePos = graph.x[nodesNumber - 1], graph.y[nodesNumber - 1]
            crossed = graph.crossObstacle(*currentNodePos, *lastNodePos)

            if not crossed:
                print("New point")
                pygame.draw.line(
                    map.map,
                    map.MapSettings["colors"]["path"],
                    currentNodePos,
                    lastNodePos,
                    map.edgeThickness,
                )

        pygame.display.update()

    iteration = 0

    while iteration < 500:
        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goalPosition)
            pygame.draw.circle(
                map.map,
                map.MapSettings["colors"]["bias"],
                (X[-1], Y[-1]),
                map.nodeRad + 2,
                0,
            )

            pygame.draw.line(
                map.map,
                map.MapSettings["colors"]["bias"],
                (X[-1], Y[-1]),
                (X[Parent[-1]], Y[Parent[-1]]),
                map.edgeThickness,
            )

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(
                map.map,
                map.MapSettings["colors"]["expand"],
                (X[-1], Y[-1]),
                map.nodeRad + 2,
                0,
            )
            pygame.draw.line(
                map.map,
                map.MapSettings["colors"]["expand"],
                (X[-1], Y[-1]),
                (X[Parent[-1]], Y[Parent[-1]]),
                map.edgeThickness,
            )

        if iteration % 5 == 0:
            pygame.display.update()

        iteration += 1

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    main()
