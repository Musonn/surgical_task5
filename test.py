import roboticstoolbox as rtb
from spatialmath import SE3
import posemath

pose1 = SE3(-0.268, -0.007, -1.179)
pose2 = SE3(-0.438, -0.205, -1.433)
tj = rtb.ctraj(pose1, pose2, 20)

print(tj)

frame = posemath.fromMatrix(tj)

print(frame)
