import math
import random
import pygame


class RRTMap:
    MapSettings = {
        "colors": {
            "background": (255, 255, 255),
            "obstacle": (0, 0, 255),
            "start": (0, 255, 0),
            "goal": (255, 0, 0),
            "path": (0, 255, 255),
            "tree": (0, 0, 0),
            "tree_point": (0, 0, 0),
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
        self.nodeThickness = 1
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

    def makeobs(self):
        obsList = []

        for i in range(0, self.obsNumber):
            obstacle = None
            startGoalColumn = (
                True  # Indicates if the obstacle is in the start or goal column
            )

            # Make sure the obstacle is not in the start or goal points
            while startGoalColumn:
                upperCorner = self.makeRandomRect()
                obstacle = pygame.Rect(
                    upperCorner, (self.obsDimensions, self.obsDimensions)
                )

                if obstacle.collidepoint(self.start) or obstacle.collidepoint(
                    self.goal
                ):
                    startGoalColumn = True
                else:
                    startGoalColumn = False

            obsList.append(obstacle)

        self.obstacles = obsList.copy()
        return obsList

    def add_node(self, posIndex, nodeX, nodeY):
        self.x.insert(posIndex, nodeX)
        # WHY IT WAS DIFFERENT ?
        self.y.append(nodeY)

    def remove_node(self, nodeIndex):
        print((len(self.x), len(self.y)))
        self.x.pop(nodeIndex)
        self.y.pop(nodeIndex)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, childIndex):
        self.parent.pop(childIndex)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, startIndex, finishIndex):
        """Messure the distance between two nodes from their indexes"""
        (startX, startY) = (self.x[startIndex], self.y[startIndex])
        (finishX, finishY) = (self.x[finishIndex], self.y[finishIndex])

        # Calculus of the hypotenuse of a right triangle
        px = float(finishX) - float(startX)
        py = float(finishY) - float(startY)
        return (px**2 + py**2) ** 0.5

    def generate_random_sample(self):
        x = random.uniform(0, self.mapWidth)
        y = random.uniform(0, self.mapHeight)

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

    def isFree(self):
        last_number_id = self.number_of_nodes() - 1

        (x, y) = (self.x[last_number_id], self.y[last_number_id])

        obs = self.obstacles.copy()

        # For each obj, verify if it has
        while len(obs) > 0:
            obstacle = obs.pop(0)
            if obstacle.collidepoint(x, y):
                self.remove_node(last_number_id)
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

                if obstacle.collidepoint(x, y):
                    return True

        return False

    def connect(self, nodeOneIdx, nodeTwoIdx):
        (x1, y1) = (self.x[nodeOneIdx], self.y[nodeOneIdx])
        (x2, y2) = (self.x[nodeTwoIdx], self.y[nodeTwoIdx])

        if self.crossObstacle(x1, x2, y1, y2):
            # If has collision, remove that node because it has not utility
            self.remove_node(nodeTwoIdx)
            return False

        else:
            self.add_edge(nodeOneIdx, nodeTwoIdx)
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

            self.remove_node(randomNode)

            if (abs(x - self.goal[0]) < maxDistance) and (
                abs(y - self.goal[1]) < maxDistance
            ):
                self.add_node(randomNode, self.goal[0], self.goal[1])
                self.goalState = randomNode
                self.goalFlag = True

            else:
                self.add_node(randomNode, x, y)

    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass

    def bias(self, goalNode):
        """
        Go through the nearest neighbor
        """
        newNode = self.number_of_nodes()

        self.add_node(newNode, goalNode[0], goalNode[1])
        nearNode = self.nearest_neighbor(newNode)
        self.step(nearNode, newNode)
        self.connect(nearNode, newNode)

        return self.x, self.y, self.parent

    def expand(self):
        """
        Go explore the map by random sample node
        """
        newNode = self.number_of_nodes()

        x, y = self.generate_random_sample()
        self.add_node(newNode, x, y)

        if self.isFree():
            nearNode = self.nearest_neighbor(newNode)
            self.step(nearNode, newNode)
            self.connect(nearNode, newNode)

        return self.x, self.y, self.parent

    def cost(self):
        pass
