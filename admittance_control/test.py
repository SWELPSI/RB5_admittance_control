import numpy as np

import roboticstoolbox as rtb
import spatialmath


from math import pi

robot = rtb.models.DH.RB5()
#机械臂信息
matrix=np.linalg.inv(robot.jacob0([0,0,pi/2,0,0,0]))
# mat_rot=matrix.Rz(180,'deg')
# print(robot.jacobe([0,pi/3,0,0,0,0]))
# print(mat_rot)
print(matrix)
# SE3
# qt = rtb.tools.trajectory.jtraj(robot.qz, robot.qr, 50)
# f=robot.rne_python(robot.qr,[0,0,0,0,0,0],[0,0,0,0,0,0],None,None)
# robot.plot([0,0,0,0,0,0])
# print(np.size(robot.qr))
# robot.dynamics_list()
# print(f)