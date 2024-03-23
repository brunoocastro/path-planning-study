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
        self.nodeRad = 0
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
            obs = obsList.pop(0)
            # Refactor to receive the draw method in the obstacle class
            # To allow multiple obstacle shapes.
            pygame.draw.rect(self.map, self.MapSettings["colors"]["obstacle"], obs)


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

    def add_node(self):
        pass

    def remove_node(self):
        pass

    def add_edge(self):
        pass

    def remove_edge(self):
        pass

    def number_of_nodes(self):
        pass

    def distance(self):
        pass

    def nearest_neighbor(self):
        pass

    def isFree(self):
        pass

    def crossObstacle(self):
        pass

    def connect(self):
        pass

    def step(self):
        pass

    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass

    def bias(self):
        pass

    def expand(self):
        pass

    def cost(self):
        pass
