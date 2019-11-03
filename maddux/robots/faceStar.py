# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib as mpl
# import pandas as pd
# from collections import OrderedDict
import numpy as np
import heapq

# note: variable ending with "face" is the coordinate of a face; variable ending with "idx" is a face label of a face

blockWidth = 0.49 # it is not 0.5 cuz this would make it a lot easier to calculate which block a face belongs to

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
    def __init__(self, startFace, goalFace, blueprint):
        self.startFace = startFace
        self.goalFace = goalFace
        self.blueprint = blueprint
        self.building_dimensions = self.blueprint.shape
        print("\nBuilding Dimensions: {}\n".format(self.building_dimensions))
        self.colors = np.array([[['#424ef5']*self.building_dimensions[2]] *
                   self.building_dimensions[1]]*self.building_dimensions[0])

        self.route = None
        # self.logger = logging.getLogger('PathPlanning')

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    # array: the current structure, startFace: the start face, goalFace: the goal face
    def faceStar(self, array, startFace, goalFace):

        neighbors = [(0, 1, 0), (0, 1, -1), (0, 1, 1),
                     (0, -1, 0), (0, -1, -1), (0, -1, 1),
                     (1, 0, 0), (1, 0, -1), (1, 0, 1),
                     (-1, 0, 0), (-1, 0, -1), (-1, 0, 1),
                     (0, 0, 0),(0, 0, 1), (0, 0, -1),
                     (1, 1, 0), (1, 1, -1), (1, 1, 1),
                     (1, -1, 0), (1, -1, -1), (1, -1, 1),
                     (-1, 1, 0), (-1, 1, -1), (-1, 1, 1),
                     (-1, -1, 0), (-1, -1, -1), (-1, -1, 1)
                     ]

        neighborFaces = [(0, 1, 0),
                     (0, -1, 0),
                     (1, 0, 0),
                     (-1, 0, 0),
                     (0, 0, 1), 
                     (0, 0, -1)
                     ]

        armReach = 1.6

        close_set = set()
        came_from = {}
        gscore = {startFace: 0}
        fscore = {startFace: self.heuristic(startFace, goalFace)}
        oheap = []

        heapq.heappush(oheap, (fscore[startFace], startFace))

        while oheap:

            currentFace = heapq.heappop(oheap)[1]

            if currentFace == goalFace:
                data = []
                while currentFace in came_from:
                    data.append(currentFace)
                    currentFace = came_from[currentFace]
                return data

            close_set.add(currentFace)
            for i, j, k in neighbors:
                currentBlock = self.get_block_idx(array, currentFace)
                neighbor = currentBlock[0] + i, currentBlock[1] + j, currentBlock[2] + k
                # for each available neighbor
                if self.within_range_huh(array,neighbor[0], neighbor[1], neighbor[2]):
                    if array[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                        continue
                else:
                    continue
                
                # for each face on neighbor
                for x, y, z in neighborFaces:
                    # if there is not a block on a face
                    if array[neighbor[0]+i][neighbor[1]+j][neighbor[2]+k] == 0:
                        neighborFace = neighbor[0] + i*blockWidth, neighbor[1] + j*blockWidth, neighbor[k] + k*blockWidth
                        tentative_g_score = gscore[currentFace] + self.heuristic(currentFace, neighborFace)
                        if neighborFace in close_set and tentative_g_score >= gscore.get(neighborFace, 0):
                            continue

                        if tentative_g_score < gscore.get(neighborFace, 0) or neighborFace not in [i[1]for i in oheap]:
                            came_from[neighborFace] = currentFace
                            gscore[neighborFace] = tentative_g_score
                            fscore[neighborFace] = tentative_g_score + \
                                self.heuristic(neighborFace, goalFace)
                            heapq.heappush(oheap, (fscore[neighborFace], neighborFace))

    def face_reachable_huh(self, array, currentFace, targetFace, armReach):
        if self.heuristic(currentFace, target) < armReach:
            currentFaceIdx = self.get_face_index(array,currentFace)
            targetFaceIdx = self.get_face_index(array,targetFace)
            if self.target_on_perpendicular_plane_huh(currentFace, targetFace, currentFaceIdx, targetFaceIdx):
                return True
            else:
                return False
        # TODO there may be potential special cases that needs to be handled with
        else:
            return False          

    def target_on_perpendicular_plane_huh(self, currentFace, targetFace, currentFaceIdx, targetFaceIdx):
        if currentFaceIdx == 'a' and targetFace[1] > currentFace[1] and targetFaceIdx == 'a' and targetFaceIdx == 'b':
            return False
        elif currentFaceIdx == 'b' and targetFace[1] < currentFace[1] and targetFaceIdx == 'a' and targetFaceIdx == 'b':
            return False
        elif currentFaceIdx == 'c' and targetFace[0] > currentFace[0] and targetFaceIdx == 'c' and targetFaceIdx == 'd':
            return False
        elif currentFaceIdx == 'd' and targetFace[0] < currentFace[0] and targetFaceIdx == 'c' and targetFaceIdx == 'd':
            return False
        elif currentFaceIdx == 'e' and targetFace[2] < currentFace[2] and targetFaceIdx == 'e' and targetFaceIdx == 'f':
            return False
        elif currentFaceIdx == 'f' and targetFace[2] > currentFace[2] and targetFaceIdx == 'e' and targetFaceIdx == 'f':
            return False
        else:
            return True

    def get_face_index(self, array, face):
        idx_x, idx_y, idx_z = self.get_block_idx(array, face)

        if self.within_range_huh(array, idx_x, idx_y, idx_z):
            if array[idx_x][idx_y][idx_z] == 0:
                return None
            else:
                if idx_x > face[0]: # face d
                    return 'd'
                elif idx_x < face[0]: # face c
                    return 'c'
                elif idx_y > face[1]: # face b
                    return 'b'
                elif idx_y < face[1]: # face a
                    return 'a'
                elif idx_z > face[2]: # face e
                    return 'e'
                elif idx_z < face[2]: # face f
                    return 'f'

    def get_block_idx(self, array, face):
        idx_x = int(round(face[0]))
        idx_y = int(round(face[1]))
        idx_z = int(round(face[2]))
        return idx_x, idx_y, idx_z

    def within_range_huh(self,array,x,y,z):
        if 0 <= x < array.shape[0] and 0 <= y < array.shape[1] and 0 <= z < array.shape[2]:
            return True
        else:
            return False

    def get_path(self):
        route = self.faceStar(self.blueprint, self.startFace, self.goalFace)
        if route is None:
            # self.logger.error("Unable to find route between points {} and {}".format(self.startFace, self.goalFace))
            raise Exception("Path planning unable to find route")
        route = route + [self.startFace]
        route = route[::-1]
        print("Path to Traverse: {}\n".format(route))
        self.route = route
        return route

    def display_path(self):
        for i in self.route:
            self.colors[i] = '#ff0000ff'

        self.colors[self.route[-1]] = '#03fc62'
        return self.colors

    # def get_traversal(self):
    #     route = self.get_path()
    #     for i in range(len(route)):
    #         block = route[i]
    #         newBlock = Block(block[0],block[1],block[2],availableFaces[i])
    #         route[i] = list(route[i])
    #         route[i].append(newBlock.get_face(0))
    #         route[i] = tuple(route[i])

    #         print("Block: {}\n".format(block))

    #     print("Path to Traverse: {}\n".format(route))

if __name__ == '__main__':
    startFace = (0.5,0,0)
    endFace = (1.5,0,0)
    bp  = np.array([
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
            [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        ])

    faceStar = PathPlanner(startFace,endFace,bp)
    faceStar.get_path()

    

