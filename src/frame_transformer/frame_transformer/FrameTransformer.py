import numpy as np
from math import cos as c
from math import sin as s
from math import pi

class FrameTransformer():


    def __init__(self) -> None:
        pass


    def SetOriginOfMissionFrame(self, x, y, z):
        self.org_ = np.array([[x], [y], [z]])

    def SetRotationOfMissionFrame(self, g, b, a):
        Rz = np.array([[c(a), -s(a), 0],
                      [s(a), c(a), 0],
                      [0, 0, 1]])
        Ry = np.array([[c(b), 0, s(b)],
                      [0, 1, 0],
                      [-s(b), 0, c(b)]])
        Rx = np.array([[1, 0, 0],
                      [0, c(g), -s(g)],
                      [0, s(g), c(g)]])
        self.rotation_ = Rz @ Ry @ Rx

    def SetIMUToBodyTransformation(self):
        self.IMU_to_body_trans_ = np.array([[c(pi/2), 0, s(pi/2)],
                                            [0, 1, 0],
                                            [-s(pi/2), 0, c(pi/2)]])


    def SetTransformationMatrix(self, init_coords, init_rotation):
        if len(init_coords) != 3 or len(init_rotation) != 3:
            raise Exception("Incorrect format of coords and rotation!")
            
    
    def TransformToMissionFrame(self, coord):
        pass
