# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib as mpl
# import pandas as pd
# from collections import OrderedDict
import numpy as np
import heapq

blockWidth == 0.5

class Block:

    def __init__(self, xPos, yPos, zPos, availableFaces):
        self.xPos = xPos
        self.yPos = yPos
        self.zPos = zPos
        self.availableFaces = availableFaces

    def update_block(self, availableFaces):
        self.availableFaces = availableFaces
    
    # previousFace is index of the selected face in the previous block in the selected path
    def get_face(self, previousFace):
        if self.availableFaces[previousFace]:
            return previousFace
        else:
            excludedFace = 0
            if previousFace == 0: # face a
                excludedFace = 1
            elif previousFace == 1: # face b
                excludedFace = 0
            elif previousFace == 2: # face c
                excludedFace = 3
            elif previousFace == 3: # face d
                excludedFace = 2
            elif previousFace == 4: # face e
                excludedFace = 5
            elif previousFace == 5: # face f
                excludedFace = 3
            
            for i in range(len(self.availableFaces)):
                minHeuristic = 10000000
                if i != excludedFace:
                    if self.availableFaces[i]: # if face is available
                        return i # TODO change this to use actual closest distance 



    
class PathPlanner:
    def __init__(self, start, goal, blueprint):
        self.start = start
        self.goal = goal
        self.blueprint = blueprint
        self.building_dimensions = self.blueprint.shape
        print("\nBuilding Dimensions: {}\n".format(self.building_dimensions))
        self.colors = np.array([[['#424ef5']*self.building_dimensions[2]] *
                   self.building_dimensions[1]]*self.building_dimensions[0])

        self.route = None
        # self.logger = logging.getLogger('PathPlanning')

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def astar(self, array, start, goal, startFace, goalFace):

        # neighbors = [(0, 1, 0), (0, 1, -1), (0, 1, 1),
        #              (0, -1, 0), (0, -1, -1), (0, -1, 1),
        #              (1, 0, 0), (1, 0, -1), (1, 0, 1),
        #              (-1, 0, 0), (-1, 0, -1), (-1, 0, 1),
        #              (0, 0, 1), (0, 0, -1),
        #              (1, 1, 0), (1, 1, -1), (1, 1, 1),
        #              (1, -1, 0), (1, -1, -1), (1, -1, 1),
        #              (-1, 1, 0), (-1, 1, -1), (-1, 1, 1),
        #              (-1, -1, 0), (-1, -1, -1), (-1, -1, 1)
        #              ]

        neighbors = [(0, 1, 0),
                     (0, -1, 0),
                     (1, 0, 0),
                     (-1, 0, 0),
                     (0, 0, 1), 
                     (0, 0, -1),
                     ]

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goalFace)}
        oheap = []

        gscore[startFace] = 0
        fscore[startFace] = self.heuristic(startFace,goalFace)
        faceHeap = []

        heapq.heappush(oheap, (fscore[start], start))

        # this implementation fails when the goal face cannot be directly reached
        heapq.heappush(faceHeap, (fscore[startFace], startFace))

        while oheap:
            current = heapq.heappop(oheap)[1]
            currentFace = heapq.heappop(faceHeap)[1]
            if current == goal:
                data = []
                while currentFace in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j, k in neighbors:
                neighbor = current[0] + i, current[1] + j, current[2] + k
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                # for each available neighbor
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:
                        if 0 <= neighbor[2] < array.shape[2]:
                            if array[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                                if i == 1: # face d
                                    pass
                                elif i == -1: # face c
                                    pass
                                elif j == 1: # face a
                                    pass
                                elif j == -1: # face b
                                    pass
                                elif k == 1: # face e
                                    pass
                                elif k == -1: # face f
                                    pass
                                neighborFace = current[0] + i*blockWidth, current[1] + j*blockWidth, current[2] + k*blockWidth
                                # add face to heap when there is no neighbor on one side
                                # TODO might be wrong to calculate g and f scores using the block itself instead of the actual face
                                tentative_face_g_score = gscore[current] + self.heuristic(current, neighborFace)
                                if tentative_face_g_score < gscore.get(neighborFace, 0) or neighborFace not in [i[1]for i in faceHeap]:
                                    came_from[neighbor] = current
                                    gscore[neighbor] = tentative_g_score
                                    fscore[neighbor] = tentative_g_score + \
                                        self.heuristic(neighbor, goalFace)
                                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                                continue
                        else:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                
                # for each face on the block
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:
                        if 0 <= neighbor[2] < array.shape[2]:
                            if array[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                                continue
                        else:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + \
                        self.heuristic(neighbor, goalFace)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

    def get_path(self):
        route = self.astar(self.blueprint, self.start, self.goal)
        if route is None:
            # self.logger.error("Unable to find route between points {} and {}".format(self.start, self.goal))
            raise Exception("Path planning unable to find route")
        route = route + [self.start]
        route = route[::-1]
        print("Path to Traverse: {}\n".format(route))
        self.route = route
        return route

    def display_path(self):
        for i in self.route:
            self.colors[i] = '#ff0000ff'

        self.colors[self.route[-1]] = '#03fc62'
        return self.colors

    def get_traversal(self):
        route = self.get_path()
        for i in range(len(route)):
            block = route[i]
            newBlock = Block(block[0],block[1],block[2],availableFaces[i])
            route[i] = list(route[i])
            route[i].append(newBlock.get_face(0))
            route[i] = tuple(route[i])

            print("Block: {}\n".format(block))

        print("Path to Traverse: {}\n".format(route))

if __name__ == '__main__':
    start = (0,0,0)
    end = (7,0,0)
    bp  = np.array([
            [[1, 0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
            [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        ])

    aStar = PathPlanner(start,end,bp)
    aStar.get_traversal()

    

