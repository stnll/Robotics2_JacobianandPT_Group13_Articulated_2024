import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH

a1 = float(input("a1 = "))*100
a2 = float(input("a2 = "))*100
a3 = float(input("a3 = "))*100

def mm_m(a):
    m = 100 #; meter = 1000mm
    return a/m

a1 = mm_m(a1)
a2 = mm_m(a2)
a3 = mm_m(a3)


# Create links
#robot variable =  DHrobot([RevoluteDH,(d,r,alpha,offset,qlim)])
#robot variable =  DHrobot([PrismaticDH,(d=0,r,alpha,offset=d,qlim)])

Articulated = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
],name='Articulated')

print(Articulated)

## Path and Trajectory Planning
# q paths
# Articulated Joint Variables = ([T1, T2, T3])
# degrees to radian

def deg_to_rad(T):
    return (T/180.0)*np.pi

## q Planned Paths
# for Articulated Jonit Variables = ([T1,T2,T3])

q0 = np.array([0,0,0]) #starting position

q1 = np.array([deg_to_rad(0),
                deg_to_rad(-30),
                deg_to_rad(-30)])  #pick position1

q2 = np.array([deg_to_rad(90),
                deg_to_rad(0),
                deg_to_rad(0)])  #revolve 90 degrees

q3 = np.array([deg_to_rad(90),
                deg_to_rad(-30),
                deg_to_rad(-30)])  #place position1

q4 = np.array([deg_to_rad(180),
                deg_to_rad(0),
                deg_to_rad(0)]) #revolve 180 degrees

q5 = np.array([deg_to_rad(180),
                deg_to_rad(-30),
                deg_to_rad(-30)]) #place position2

q6 = np.array([deg_to_rad(270),
                deg_to_rad(0),
                deg_to_rad(0)]) #revolve 270 degrees

q7 = np.array([deg_to_rad(270),
                deg_to_rad(-30),
                deg_to_rad(-30)]) #place position3

q8 = np.array([deg_to_rad(360),
                deg_to_rad(0),
                deg_to_rad(0)]) #end position

steps = 20
# planned Trajectories
traj1 = rtb.jtraj(q0,q1,steps) # pick 1 start
traj2 = rtb.jtraj(q1,q0,steps) # up 1
traj3 = rtb.jtraj(q0,q2,steps) # rotate 1
traj4 = rtb.jtraj(q2,q3,steps) # place 1
traj5 = rtb.jtraj(q3,q2,steps) # retract 1
traj6 = rtb.jtraj(q2,q0,steps) # return 1
traj7 = rtb.jtraj(q0,q1,steps) # pick 2
traj8 = rtb.jtraj(q1,q0,steps) # up 2
traj9 = rtb.jtraj(q0,q4,steps) # rotate 2
traj10 = rtb.jtraj(q4,q5,steps) # place 2
traj11 = rtb.jtraj(q5,q4,steps) # retract 2
traj12 = rtb.jtraj(q4,q0,steps) # return 2
traj13 = rtb.jtraj(q0,q1,steps) # pick 3
traj14 = rtb.jtraj(q1,q0,steps) # up 3
traj15 = rtb.jtraj(q0,q6,steps) # rotate 3
traj16 = rtb.jtraj(q6,q7,steps) # place 3
traj17 = rtb.jtraj(q7,q6,steps) # retract 3
traj18 = rtb.jtraj(q6,q8,steps) # return 3 end

#plot scale
x1 = -50
x2 = 50
y1 = -50
y2 = 50
z1 = 0
z2 = 50

# Path and Trajectory plotting
Articulated.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj2.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj3.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj4.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj5.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj6.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj7.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj8.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj9.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj10.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj11.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj12.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj13.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj14.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj15.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj16.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj17.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj18.q,limits=[x1, x2, y1, y2, z1, z2], block=True)