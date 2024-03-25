import pygame
from RRTBase import RRTGraph, RRTMap


def main():
    mapDimensions = (600, 600)
    startPosition = (50, 50)
    goalPosition = (550, 550)

    obstacleSize = 50
    obstacleNumber = 50

    pygame.init()

    map = RRTMap(
        startPosition, goalPosition, mapDimensions, obstacleSize, obstacleNumber
    )
    graph = RRTGraph(
        startPosition, goalPosition, mapDimensions, obstacleSize, obstacleNumber
    )

    obstacles = graph.generateObstacles()
    map.drawMap(obstacles)

    # count = 0
    # range = 2
    # while count < range:

    #     count += 1
    #     randomPos = graph.generateRandomPosition()
    #     freePosition = graph.isFree(*randomPos)

    #     print("isFree", freePosition)
    #     if not freePosition:
    #         continue
    #     nodesNumber = graph.numberOfNodes()
    #     graph.addNode(nodesNumber, *randomPos)
    #     graph.addEdge(nodesNumber - 1, nodesNumber)

    #     # If has not collision, draw
    #     pygame.draw.circle(
    #         map.map,
    #         map.MapSettings["colors"]["path"],
    #         (graph.x[nodesNumber], graph.y[nodesNumber]),
    #         map.nodeRad,
    #         map.nodeThickness,
    #     )

    #     currentNodePos = graph.x[nodesNumber], graph.y[nodesNumber]
    #     lastNodePos = graph.x[nodesNumber - 1], graph.y[nodesNumber - 1]
    #     crossed = graph.crossObstacle(*currentNodePos, *lastNodePos)

    #     if not crossed:
    #         print("New point")
    #         pygame.draw.line(
    #             map.map,
    #             map.MapSettings["colors"]["path"],
    #             currentNodePos,
    #             lastNodePos,
    #             map.edgeThickness,
    #         )

    #     pygame.display.update()

    iteration = 0
    bias = 0
    expand = 0

    numberOfPoints = 500

    lastDrawPos = None

    while iteration < numberOfPoints:
        if iteration % 10 == 0 and False:
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
            print("bias")
            bias += 1

        else:
            expand += 1
            X, Y, Parent = graph.expand()
            # print("expand", X, Y, Parent)

            # newDrawPos = (X[-1], Y[-1])

            # if newDrawPos != lastDrawPos:
            #     lastDrawPos = newDrawPos

            #     pygame.draw.circle(
            #         map.map,
            #         map.MapSettings["colors"]["expand"],
            #         newDrawPos,
            #         map.nodeRad + 2,
            #         0,
            #     )
            #     pygame.draw.line(
            #         map.map,
            #         map.MapSettings["colors"]["expand"],
            #         newDrawPos,
            #         (X[Parent[-1]], Y[Parent[-1]]),
            #         map.edgeThickness,
            #     )

        if iteration % 5 == 0:
            pygame.display.update()

        iteration += 1
        if iteration == numberOfPoints:
            X, Y, Parent = graph.x, graph.y, graph.parent
            print("Data", X, Y, Parent)
            for Node in range(0, len(X) - 1):
                # pygame.draw.line(
                #     map.map,
                #     map.MapSettings["colors"]["expand"],
                #     (X[Node], Y[Node]),
                #     (X[Node + 1], Y[Node + 1]),
                #     map.edgeThickness,
                # )
                pygame.draw.circle(
                    map.map,
                    map.MapSettings["colors"]["expand"],
                    (X[Parent[Node]], Y[Parent[Node]]),
                    map.nodeRad + 2,
                    0,
                )
                pygame.draw.line(
                    map.map,
                    map.MapSettings["colors"]["expand"],
                    (X[Parent[Node]], Y[Parent[Node]]),
                    (X[Parent[-1]], Y[Parent[-1]]),
                    map.edgeThickness,
                )

    print("bias: ", bias)
    print("expand: ", expand)

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    main()
