import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

a1 = float(input("a1 = ")) * 100
a2 = float(input("a2 = ")) * 100
a3 = float(input("a3 = ")) * 100

def mm_m(a):
    m = 100  # meter = 1000mm
    return a / m

a1 = mm_m(a1)
a2 = mm_m(a2)
a3 = mm_m(a3)

# Create links

Articulated = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
],name='Articulated')

print(Articulated)

    ## Path and Trajectory Planning

    ## q Planned Paths
    # for Articulated Jonit Variables = ([x,y,z])
q0 = SE3([0,0,0])
q1 = SE3([30,
            -30,
            10]) #start

q2 = SE3([30,
            0,
            10])

q3 = SE3([30,
            30,
            10])

q4 = SE3([0,
            30,
            10])

q5 = SE3([-30,
            30,
            10])

q6 = SE3([-30,
            0,
            10])

q7 = SE3([-30,
            -30,
            10])

q8 = SE3([0,
            -30,
            10])

q9 = SE3([30,
            -30,
            10]) #end


steps = 20  # Number of steps for each trajectory

def cartesian_traj(start, end, steps):
    path = []
    for p in np.linspace(0, 1, steps):
    # Interpolate between start and end SE3 objects
        w = start.interp(end, p)
    # Solve the inverse kinematics for the interpolated position
        q = Articulated.ikine_LM(w).q
        path.append(q)
    return np.array(path)

traj1 = cartesian_traj(q0, q1, steps)
traj2 = cartesian_traj(q1, q2, steps)
traj3 = cartesian_traj(q2, q3, steps)
traj4 = cartesian_traj(q3, q4, steps)
traj5 = cartesian_traj(q4, q5, steps)
traj6 = cartesian_traj(q5, q6, steps)
traj7 = cartesian_traj(q6, q7, steps)
traj8 = cartesian_traj(q7, q8, steps)
traj9 = cartesian_traj(q8, q9, steps)
traj10 = cartesian_traj(q9, q0, steps)


#plot scale
x1 = -50
x2 = 50
y1 = -50
y2 = 50
z1 = 0
z2 = 50

# Plot the trajectories
Articulated.plot(traj1, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj2, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj3, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj4, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj5, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj6, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj7, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj8, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj9, limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj10, limits=[x1, x2, y1, y2, z1, z2], block=True)