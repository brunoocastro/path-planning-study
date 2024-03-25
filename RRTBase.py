import math
import random
import pygame


class RRTMap:
    MapSettings = {
        "colors": {
            "background": (130, 130, 200),
            "obstacle": (0, 0, 0),
            # "obstacle": (255, 10, 23),
            "start": (0, 255, 0),
            "goal": (255, 0, 0),
            "path": (0, 255, 255),
            "tree": (0, 0, 0),
            "tree_point": (0, 55, 55),
            "bias": (255, 0, 255),
            "expand": (255, 255, 0),
        }
    }

    def __init__(
        self, start, goal, MapDimensions, ObstaclesDimensions, ObstaclesNumber
    ):
        self.start = start
        self.goal = goal
        self.MapSettings["dimensions"] = MapDimensions
        self.MapSettings["height"], self.MapSettings["width"] = self.MapSettings[
            "dimensions"
        ]

        # Pygame Windows settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode(
            (self.MapSettings["width"], self.MapSettings["height"])
        )
        self.map.fill(self.MapSettings["colors"]["background"])
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsDimensions = ObstaclesDimensions
        self.obsNumber = ObstaclesNumber

    def drawMap(self, obstacles):
        # Draw start point
        pygame.draw.circle(
            self.map,
            self.MapSettings["colors"]["start"],
            self.start,
            self.nodeRad + 5,
            0,
        )  # A filled circle with default rad + 5 - Start

        # Draw goal point
        pygame.draw.circle(
            self.map,
            self.MapSettings["colors"]["goal"],
            self.goal,
            self.nodeRad + 20,
            1,
        )  # A filled circle with default rad + 20 - Goal

        self.drawObs(obstacles)

    def drawPath(self):
        pass

    def drawObs(self, obstacles):
        obsList = obstacles.copy()
        while len(obsList) > 0:
            obstacle = obsList.pop(0)
            # Refactor to receive the draw method in the obstacle class
            # To allow multiple obstacle shapes.
            pygame.draw.rect(self.map, self.MapSettings["colors"]["obstacle"], obstacle)


class RRTGraph:
    def __init__(self, start, goal, MapDimensions, ObstacleDimensions, ObstaclesNumber):
        (startX, startY) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.mapHeight, self.mapWidth = MapDimensions

        # How to store the nodes - Refactor to a instance of "Node" containing X, Y and Parents
        self.x = []
        self.y = []
        self.parent = []

        # # Initialize tree
        self.x.append(startX)
        self.y.append(startY)
        self.parent.append(0)

        # Create the obstacles
        self.obstacles = []
        self.obsDimensions = ObstacleDimensions
        self.obsNumber = ObstaclesNumber

        # Path
        self.goalState = None
        self.path = []

    def makeRandomRect(self):
        """
        Generate a random (X,Y) coordinates (upper and left corner of
        the rectangle) of a new rect obstacle
        """
        upperCornerX = int(random.uniform(0, self.mapWidth - self.obsDimensions))
        upperCornerY = int(random.uniform(0, self.mapHeight - self.obsDimensions))

        return (upperCornerX, upperCornerY)

    def generateObstacles(
        self,
    ):
        obsList = []

        for i in range(0, self.obsNumber):
            obstacle = None
            # Indicates if the obstacle is in the start or goal column
            blockedPositionCollision = True

            # Make sure the obstacle is not in the start or goal points
            while blockedPositionCollision:
                upperCorner = self.makeRandomRect()
                obstacle = pygame.Rect(
                    upperCorner, (self.obsDimensions, self.obsDimensions)
                )

                if obstacle.collidepoint(self.start) or obstacle.collidepoint(
                    self.goal
                ):
                    blockedPositionCollision = True
                else:
                    blockedPositionCollision = False

            obsList.append(obstacle)

        self.obstacles = obsList.copy()
        print("Amount of obstacles", self.obstacles)
        return obsList

    def addNode(self, index, X, Y):
        self.x.insert(index, X)
        # WHY IT WAS DIFFERENT ?
        self.y.append(Y)

    def removeNode(self, index):
        self.x.pop(index)
        self.y.pop(index)

    def addEdge(self, parent, child):
        print("Add edge", child, "to", parent)
        self.parent.insert(child, parent)

    def removeEdge(self, childIndex):
        self.parent.pop(childIndex)

    def numberOfNodes(self):
        return len(self.x)

    def distance(self, startIndex, finishIndex):
        """Messure the distance between two nodes from their indexes"""
        (startX, startY) = (self.x[startIndex], self.y[startIndex])
        (finishX, finishY) = (self.x[finishIndex], self.y[finishIndex])

        # Calculus of the hypotenuse of a right triangle
        px = (float(startX) - float(finishX)) ** 2
        py = (float(startY) - float(finishY)) ** 2

        hypotenuse = (px + py) ** 0.5

        return hypotenuse

    def generateRandomPosition(self):
        x = int(random.uniform(0, self.mapWidth))
        y = int(random.uniform(0, self.mapHeight))

        return (x, y)

    def nearest_neighbor(self, node):
        minDistance = self.distance(0, node)

        nearestNode = 0

        for i in range(0, node):
            distance = self.distance(i, node)

            if distance < minDistance:
                minDistance = distance
                nearestNode = i

        return nearestNode

    def isFree(self, x, y):
        obs = self.obstacles.copy()

        # For each obj, verify if it has
        while len(obs) > 0:
            obstacle = obs.pop(0)
            if obstacle.collidepoint(x, y):
                print("Collision Detected. Is not free")
                # self.removeNode(numberOfNodes)
                return False

        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()

        while len(obs) > 0:
            obstacle = obs.pop(0)

            # Check for collisions
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)

                # print("obs", obstacle, obstacle.collidepoint(x, y))
                if self.isFree(x, y):
                    # print("Collision Detected. Removing node", self.numberOfNodes() - 1)
                    # self.numberOfNodes()
                    # pygame.draw.circle(self.map, (0, 0, 255), (x, y), 5, 0)
                    return True

        return False

    def connect(self, nodeOneIdx, nodeTwoIdx):
        (x1, y1) = (self.x[nodeOneIdx], self.y[nodeOneIdx])
        (x2, y2) = (self.x[nodeTwoIdx], self.y[nodeTwoIdx])

        cross = self.crossObstacle(x1, x2, y1, y2)

        if cross:
            print("Must remove because cross obtacle")
            # If has collision, remove that node because it has not utility
            self.removeNode(nodeTwoIdx)
            return False

        else:
            self.addEdge(nodeOneIdx, nodeTwoIdx)
            return True

    def step(self, nearNode, randomNode, maxDistance=35):
        distance = self.distance(nearNode, randomNode)

        if distance > maxDistance:
            # u = maxDistance / distance

            (nearX, nearY) = (self.x[nearNode], self.y[nearNode])
            (randX, randY) = (self.x[randomNode], self.y[randomNode])

            (px, py) = (randX - nearX, randY - nearY)

            theta = math.atan2(py, px)

            (x, y) = (
                int(nearX + maxDistance * math.cos(theta)),
                int(nearY + maxDistance * math.sin(theta)),
            )

            self.removeNode(randomNode)

            # Check if the new created node found the goal
            if (abs(x - self.goal[0]) < maxDistance) and (
                abs(y - self.goal[1]) < maxDistance
            ):
                self.addNode(randomNode, self.goal[0], self.goal[1])
                self.goalState = randomNode
                self.goalFlag = True

            else:
                print("Random node", randomNode, "x", x, "y", y)
                self.addNode(randomNode, x, y)

    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass

    def bias(self, goalNode):
        """
        Go through the nearest neighbor
        """
        newNode = self.numberOfNodes()
        print("Number of Nodes", newNode, "goal node", goalNode)

        self.addNode(newNode, goalNode[0], goalNode[1])
        nearNode = self.nearest_neighbor(newNode)
        self.step(nearNode, newNode)
        self.connect(nearNode, newNode)

        return self.x, self.y, self.parent

    def expand(self):
        """
        Go explore the map by random sample node
        """
        newNode = self.numberOfNodes()

        x, y = self.generateRandomPosition()
        if self.isFree(x, y):
            nearNode = self.nearest_neighbor(newNode)
            cross = self.crossObstacle(self.x[nearNode], x, self.y[nearNode], y)
            print("Cross", cross)
            # self.removeNode(newNode)
            if not cross:
                self.addNode(newNode, x, y)
                self.step(nearNode, newNode)
                self.connect(nearNode, newNode)

        return self.x, self.y, self.parent

    def cost(self):
        pass
