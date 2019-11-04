import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib as mpl
# import pandas as pd
# from collections import OrderedDict
import numpy as np
import heapq

# note: variable ending with "face" is the coordinate of a face; variable ending with "idx" is a face label of a face

blockWidth = 0.49 # it is not 0.5 cuz this would make it a lot easier to calculate which block a face belongs to

class BlockFace:

    def __init__(self, xPos, yPos, zPos, face):
        self.xPos = xPos
        self.yPos = yPos
        self.zPos = zPos
        self.face = face
    
    def get_face_coordinate(self):
        coordinate = [self.xPos, self.yPos, self.zPos]
        if self.face == 'a':
            coordinate[1] = coordinate[1] - blockWidth
        elif self.face == 'b':
            coordinate[1] = coordinate[1] + blockWidth
        elif self.face == 'c':
            coordinate[0] = coordinate[0] - blockWidth
        elif self.face == 'd':
            coordinate[0] = coordinate[0] + blockWidth
        elif self.face == 'e':
            coordinate[2] = coordinate[2] + blockWidth
        elif self.face == 'f':
            coordinate[2] = coordinate[2] - blockWidth
        else:
            return None
        return tuple(coordinate)
    


class FaceStar:
    def __init__(self, startFace, goalFace, blueprint):

        self.startFace = startFace.get_face_coordinate()
        if not self.startFace:
            raise Exception("Start face is invalid")
        self.goalFace = goalFace.get_face_coordinate()
        if not self.goalFace:
            raise Exception("Goal face is invalid")

        self.bp = blueprint
        self.building_dimensions = self.bp.shape
        print("\nBuilding Dimensions: {}\n".format(self.building_dimensions))
        self.colors = np.array([[[(0,0,1,0.3)]*self.building_dimensions[2]] * self.building_dimensions[1]]*self.building_dimensions[0], )

        # self.logger = logging.getLogger('PathPlanning')

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    # array: the current structure, startFace: the start face, goalFace: the goal face
    def faceStar(self, startFace, goalFace):

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
                currentBlock = self.get_block_idx(currentFace)
                neighbor = currentBlock[0] + i, currentBlock[1] + j, currentBlock[2] + k
                # for each available neighbor
                if self.within_range_huh(neighbor[0], neighbor[1], neighbor[2]):
                    if self.bp[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                        continue
                else:
                    continue
                
                # for each face on neighbor
                for x, y, z in neighborFaces:
                    # if there is not a block on a face
                    nextNeighbor = neighbor[0]+x, neighbor[1]+y, neighbor[1]+z
                    if self.within_range_huh(nextNeighbor[0], nextNeighbor[1], nextNeighbor[2]):
                        if self.bp[nextNeighbor[0]][nextNeighbor[1]][nextNeighbor[2]] == 0:
                            neighborFace = neighbor[0] + x*blockWidth, neighbor[1] + y*blockWidth, neighbor[2] + z*blockWidth
                            tentative_g_score = gscore[currentFace] + self.heuristic(currentFace, neighborFace)

                            if neighborFace in close_set and tentative_g_score >= gscore.get(neighborFace, 0):
                                continue

                            if tentative_g_score < gscore.get(neighborFace, 0) or neighborFace not in [i[1]for i in oheap]:
                                came_from[neighborFace] = currentFace
                                gscore[neighborFace] = tentative_g_score
                                fscore[neighborFace] = tentative_g_score + self.heuristic(neighborFace, goalFace)
                                heapq.heappush(oheap, (fscore[neighborFace], neighborFace))

    def face_reachable_huh(self, currentFace, targetFace, armReach):
        if self.heuristic(currentFace, targetFace) < armReach:
            currentFaceIdx = self.get_face_index(currentFace)
            targetFaceIdx = self.get_face_index(targetFace)
            if self.target_on_perpendicular_plane_huh(currentFace, targetFace, currentFaceIdx, targetFaceIdx):
                return True
            else:
                return False
        # TODO there may be potential special cases that needs to be handled with
        else:
            return False          

    def target_on_perpendicular_plane_huh(self, currentFace, targetFace, currentFaceIdx, targetFaceIdx):
        if currentFaceIdx == 'a' and targetFace[1] > currentFace[1] and (targetFaceIdx == 'a' or targetFaceIdx == 'b'):
            return False
        elif currentFaceIdx == 'b' and targetFace[1] < currentFace[1] and (targetFaceIdx == 'a' or targetFaceIdx == 'b'):
            return False
        elif currentFaceIdx == 'c' and targetFace[0] > currentFace[0] and (targetFaceIdx == 'c' or targetFaceIdx == 'd'):
            return False
        elif currentFaceIdx == 'd' and targetFace[0] < currentFace[0] and (targetFaceIdx == 'c' or targetFaceIdx == 'd'):
            return False
        elif currentFaceIdx == 'e' and targetFace[2] < currentFace[2] and (targetFaceIdx == 'e' or targetFaceIdx == 'f'):
            return False
        elif currentFaceIdx == 'f' and targetFace[2] > currentFace[2] and (targetFaceIdx == 'e' or targetFaceIdx == 'f'):
            return False
        else:
            return True

    def get_face_index(self, face):
        idx_x, idx_y, idx_z = self.get_block_idx(face)

        if self.within_range_huh(idx_x, idx_y, idx_z):
            if self.bp[idx_x][idx_y][idx_z] == 0:
                return None
            else:
                if idx_x < face[0]: # face d
                    return 'd'
                elif idx_x > face[0]: # face c
                    return 'c'
                elif idx_y < face[1]: # face b
                    return 'b'
                elif idx_y > face[1]: # face a
                    return 'a'
                elif idx_z < face[2]: # face e
                    return 'e'
                elif idx_z > face[2]: # face f
                    return 'f'

    def get_block_idx(self, face):
        idx_x = int(round(face[0]))
        idx_y = int(round(face[1]))
        idx_z = int(round(face[2]))
        return idx_x, idx_y, idx_z

    def within_range_huh(self,x,y,z):
        if 0 <= x < self.bp.shape[0] and 0 <= y < self.bp.shape[1] and 0 <= z < self.bp.shape[2]:
            return True
        else:
            return False

    def get_path(self):
        route = self.faceStar(self.startFace, self.goalFace)
        if route is None:
            # self.logger.error("Unable to find route between points {} and {}".format(self.startFace, self.goalFace))
            raise Exception("Path planning unable to find route")
        route = route + [self.startFace]
        route = route[::-1]
        print("Path to Traverse: {}\n".format(route))
        self.route = route
        return route

    # def display_path(self):
    #     for i in self.route:
    #         self.colors[i] = '#ff0000ff'

    #     self.colors[self.route[-1]] = '#03fc62'
    #     return self.colors

    def display_path(self, path):
        npPath = np.array(path)
        npPath = np.add(npPath,0.5)

        fig = plt.figure(figsize=(12, 12))
        ax = Axes3D(fig)
        ax.voxels(bp1, facecolors=faceStarPlanner.colors, edgecolors='gray', zorder=0)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)

        ax.plot(npPath[:,0],npPath[:,1],npPath[:,2],c='r',marker='o',markersize=25)

        plt.show()


if __name__ == '__main__':
    startFace = BlockFace(0,0,0,'e')
    endFace = BlockFace(7,2,2,'b')
    bp1  = np.array([
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            [[1, 0, 0], [1, 0, 0], [1, 1, 1], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        ])

    bp2 = np.array([
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
    ])
    

    faceStarPlanner = FaceStar(startFace,endFace,bp1)

    path = faceStarPlanner.get_path()
    # path = [(0, 0, 0.49), (1.0, 0.0, 0.49), (2.0, 0.49, 0.0), (3.0, 0.49, 0.0), (4.0, 0.49, 0.0), (5.0, 0.49, 0.0), (6.0, 0.49, 0.0), (6.51, 1.0, 0.0), (7.0, 2.49, 0.0)]
    faceStarPlanner.display_path(path)

    


    

