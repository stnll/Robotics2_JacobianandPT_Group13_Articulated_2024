import tkinter as tk
from tkinter import*
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import math
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib
matplotlib.use('TkAgg')

#Create GUI Window with title
gggui = Tk()
gggui.title('Articulated Calculator')
gggui.resizable(False,False)
gggui.configure(bg='Navy')


def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0,END)

    T1_E.delete(0,END)
    T2_E.delete(0,END)
    T3_E.delete(0,END)

    Z_E.delete(0,END)
    Y_E.delete(0,END)
    X_E.delete(0,END)

def f_k():
#link lengths in mm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

#joint variables in mm
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    T3 = float(T3_E.get())

# degree to radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi
    T3 = (T3/180.0)*np.pi

# Parametric Table (theta, alpha, r, d)
    PT = [[T1,(90.0/180.0)*np.pi,0,a1],
          [T2,(0.0/180.0)*np.pi,a2,0],
          [T3,(0.0/180.0)*np.pi,a3,0]]

# HTM formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    H0_1 = np.matrix (H0_1)
    H1_2 = np.matrix (H1_2)
    H2_3 = np.matrix (H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)

    X0_3 = H0_3[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3*100,3))

            ##Jacobian Matrix Program

        #Jacobian window

    J_sw = Toplevel()
    J_sw.title('Velocity Calculator')
    J_sw.resizable(False,False)

    #1. Linear/Prismatic Vectors
    Z_1 = [[0],
            [0],
            [1]] # The [0,0,1] vector
    
    #Row 1 to 3, Column 1
    J1a = [[1,0,0],
            [0,1,0],
            [0,0,1]] #R0_0
    J1a = np.dot(J1a,Z_1)

    J1b_1 = H0_3[0:3,3]
    J1b_1 = np.matrix (J1b_1)

    J1b_2 = [[0],[0],[0]]
    J1b_2 = np.matrix (J1b_2)

    J1b = J1b_1 - J1b_2

    J1 = [[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
            [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
            [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]
    
    J1 = np.matrix(J1)

    #Row 1 to 3, Column 2
    J2a = H0_1[0:3,0:3] #R0_1
    J2a = np.dot (J2a,Z_1)

    J2b_1 = H0_3[0:3,3]
    J2b_1 = np.matrix (J2b_1)

    J2b_2 = H0_1[0:3,3]
    J2b_2 = np.matrix (J2b_2)

    J2b = J2b_1 - J2b_2

    J2 = [[(J2a[1,0]*J2b[2,0])-(J2a[2,0]*J2b[1,0])],
            [(J2a[2,0]*J2b[0,0])-(J2a[0,0]*J2b[2,0])],
            [(J2a[0,0]*J2b[1,0])-(J2a[1,0]*J2b[0,0])]]
    
    J2 = np.matrix(J2)

    #Row 1 to 3, Column 3
    J3a = H0_2[0:3,0:3]
    J3a = np.dot (J3a,Z_1)

    J3b_1 = H0_3[0:3,3:]
    J3b_1 = np.matrix (J3b_1)

    J3b_2 = H0_2[0:3,3:]
    J3b_2 = np.matrix (J3b_2)

    J3b = J3b_1 - J3b_2

    J3 = [[(J3a[1,0]*J3b[2,0])-(J3a[2,0]*J3b[1,0])],
            [(J3a[2,0]*J3b[0,0])-(J3a[0,0]*J3b[2,0])],
            [(J3a[0,0]*J3b[1,0])-(J3a[1,0]*J3b[0,0])]]
    
    J3 = np.matrix(J3)

    #2. Rotation/Orientation Vectors

    #Row 4 to 6, Column 1
    J4 = J1a
    J4 = np.matrix(J4)

    #Row 4 to 6, Column 2
    J5 = J2a
    J5 = np.matrix(J5)

    #Row 4 to 6, Column 3
    J6 = J3a
    J6 = np.matrix(J6)

    #3. Concatenated Jacobian Matrix
    JM1 = np.concatenate((J1,J2,J3),1)
    JM2 = np.concatenate((J4,J5,J6),1)

    J = np.concatenate((JM1,JM2),0)
    J = np.matrix(J)

    #Velocity Slide Update
    def upd_velo():
            T1p = T1_slider.get()
            T2p = T2_slider.get()
            T3p = T3_slider.get()

            q = np.array([[T1p],[T2p],[T3p]])
            E = np.dot(J,q)

            xp_e = E[0,0]
            x_entry.delete(0,END)
            x_entry.insert(0,str(xp_e))
            
            yp_e = E[1,0]
            y_entry.delete(0,END)
            y_entry.insert(0,str(yp_e))

            zp_e = E[2,0]
            z_entry.delete(0,END)
            z_entry.insert(0,str(zp_e))

            ωx_e = E[3,0]
            ωx_entry.delete(0,END)
            ωx_entry.insert(0,str(ωx_e))

            ωy_e = E[4,0]
            ωy_entry.delete(0,END)
            ωy_entry.insert(0,str(ωy_e))

            ωz_e = E[5,0]
            ωz_entry.delete(0,END)
            ωz_entry.insert(0,str(ωz_e))

    #Jacobian Sliders
    T1_velo = Label(J_sw,text=('θ1* ='),font=(5))
    T1_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
    T1_unit = Label(J_sw,text=('rad/s'),font=(5))

    T2_velo = Label(J_sw,text=('θ2* ='),font=(5))
    T2_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
    T2_unit = Label(J_sw,text=('rad/s'),font=(5))

    T3_velo = Label(J_sw,text=('θ3* ='),font=(5))
    T3_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
    T3_unit = Label(J_sw,text=('rad/s'),font=(5))

    T1_velo.grid(row=0,column=0)
    T1_slider.grid(row=0,column=1)
    T1_unit.grid(row=0,column=2)

    T2_velo.grid(row=1,column=0)
    T2_slider.grid(row=1,column=1)
    T2_unit.grid(row=1,column=2)

    T3_velo.grid(row=2,column=0)
    T3_slider.grid(row=2,column=1)
    T3_unit.grid(row=2,column=2)

    #Jacobian Entries and Labels
    x_velo = Label(J_sw,text=('x* ='),font=(5))
    x_entry = Entry(J_sw,width=10,font=(10))
    x_unit = Label(J_sw,text=('cm/s'),font=(5))
    x_velo.grid(row=3,column=0)
    x_entry.grid(row=3,column=1)
    x_unit.grid(row=3,column=2)

    y_velo = Label(J_sw,text=('y* ='),font=(5))
    y_entry = Entry(J_sw,width=10,font=(10))
    y_unit = Label(J_sw,text=('cm/s'),font=(5))
    y_velo.grid(row=4,column=0)
    y_entry.grid(row=4,column=1)
    y_unit.grid(row=4,column=2)

    z_velo = Label(J_sw,text=('z* ='),font=(5))
    z_entry = Entry(J_sw,width=10,font=(10))
    z_unit = Label(J_sw,text=('cm/s'),font=(5))
    z_velo.grid(row=5,column=0)
    z_entry.grid(row=5,column=1)
    z_unit.grid(row=5,column=2)

    ωx_velo = Label(J_sw,text=('ωx ='),font=(5))
    ωx_entry = Entry(J_sw,width=10,font=(10))
    ωx_unit = Label(J_sw,text=('rad/s'),font=(5))
    ωx_velo.grid(row=6,column=0)
    ωx_entry.grid(row=6,column=1)
    ωx_unit.grid(row=6,column=2)

    ωy_velo = Label(J_sw,text=('ωy ='),font=(5))
    ωy_entry = Entry(J_sw,width=10,font=(10))
    ωy_unit = Label(J_sw,text=('rad/s'),font=(5))
    ωy_velo.grid(row=7,column=0)
    ωy_entry.grid(row=7,column=1)
    ωy_unit.grid(row=7,column=2)

    ωz_velo = Label(J_sw,text=('ωz ='),font=(5))
    ωz_entry = Entry(J_sw,width=10,font=(10))
    ωz_unit = Label(J_sw,text=('rad/s'),font=(5))
    ωz_velo.grid(row=8,column=0)
    ωz_entry.grid(row=8,column=1)
    ωz_unit.grid(row=8,column=2)

    #Update Button
    update = Button(J_sw,text='Update',bg='green',fg='white',command=upd_velo)
    update.grid(row=9,column=0)

#Create links
#[robot_value]=DHRobot([RevoluteDH(d,r,alpha,offset)])

    Articulated = DHRobot([
        RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])],name='Articulated')

    q0 = np.array([0,0,0])
    q1 = np.array([T1,T2,T3])

    traj1 = rtb.jtraj(q0,q1,20)
    traj2 = rtb.jtraj(q1,q0,20)
    
        #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5
    
    #Plot Command
    Articulated.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
    Articulated.plot(traj2.q,limits=[x1, x2, y1, y2, z1, z2],block=True)

def i_k():
#link lengths in mm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

#position vector in mm
    x0_3 = float(X_E.get())
    y0_3 = float(Y_E.get())
    z0_3 = float(Z_E.get())

    if x0_3 == 0:
         TH1 = np.pi/2 if y0_3 > 0 else -np.pi/2
    else:
         TH1 = np.arctan(y0_3/x0_3) #Solution1

    r1 = np.sqrt(y0_3**2 + x0_3**2) #Solution2
    r2 = z0_3-a1 #Solution3

    if r1 == 0:
         phi1 = np.pi/2 if r2 > 0 else - np.pi/2
    else:
         phi1 = np.arctan(r2/r1) #Solution4

    r3 = np.sqrt(r2**2+r1**2) #Solution5

    phi2 = np.arccos(np.clip((a3**2-a2**2-r3**2)/(-2*a2*r3),-1,1)) #Solution6

    TH2 = phi1+phi2 #Solution7

    phi3 = np.arccos(np.clip((r3**2-a2**2-a3**2)/(-2*a2*a3),-1,1)) #Solution8

    TH3 = phi3-np.pi #Solution9

    T1_E.delete(0,END)
    T1_E.insert(0,np.around(TH1*180/np.pi,3))

    T2_E.delete(0,END)
    T2_E.insert(0,np.around(TH2*180/np.pi,3))

    T3_E.delete(0,END)
    T3_E.insert(0,np.around(TH3*180/np.pi,3))

#Create links
#[robot_value]=DHRobot([RevoluteDH(d,r,alpha,offset)])

    Articulated = DHRobot([
        RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])],name='Articulated')
    
    q0 = np.array([0,0,0])
    q1 = np.array([TH1,TH2,TH3])
    

    traj1 = rtb.jtraj(q0,q1,20)
    traj2 = rtb.jtraj(q1,q0,20)
    
    #plot scale
    x1 = -0.5
    x2 = 0.5
    y1 = -0.5
    y2 = 0.5
    z1 = 0.0
    z2 = 0.5
    
    #Plot Command
    Articulated.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
    Articulated.plot(traj2.q,limits=[x1, x2, y1, y2, z1, z2],block=True)

    print(Articulated)

def p_p():
    a1 = 20
    a2 = 30
    a3 = 10

    Articulated = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
],name='Articulated')


## Path and Trajectory Planning

## q Planned Paths
# for Articulated Jonit Variables = ([T1,T2,T3])

    q0 = np.array([0,0,0])

    q1 = np.array([0,
                    -np.pi/6,
                    -np.pi/6])  #pick1

    q2 = np.array([np.pi/2,
                    0,
                    0])  #revolve1

    q3 = np.array([np.pi/2,
                    -np.pi/6,
                    -np.pi/6])  #place1

    q4 = np.array([np.pi,
                    0,
                    0]) #revolve2

    q5 = np.array([np.pi,
                    -np.pi/6,
                    -np.pi/6]) #pick 3

    q6 = np.array([(3/2)*np.pi,
                    0,
                    0]) #revolve3

    q7 = np.array([(3/2)*np.pi,
                    -np.pi/6,
                    -np.pi/6]) #place3

    q8 = np.array([2*np.pi,
                    0,
                    0]) #revolve4
    
    steps = 20  # Number of steps for each trajectory


    # planned Trajectories
    traj1 = rtb.jtraj(q0,q1,steps)
    traj2 = rtb.jtraj(q1,q0,steps)
    traj3 = rtb.jtraj(q0,q2,steps)
    traj4 = rtb.jtraj(q2,q3,steps)
    traj5 = rtb.jtraj(q3,q2,steps)
    traj6 = rtb.jtraj(q2,q0,steps)
    traj7 = rtb.jtraj(q0,q1,steps)
    traj8 = rtb.jtraj(q1,q0,steps)
    traj9 = rtb.jtraj(q0,q4,steps)
    traj10 = rtb.jtraj(q4,q5,steps)
    traj11 = rtb.jtraj(q5,q4,steps)
    traj12 = rtb.jtraj(q4,q0,steps)
    traj13 = rtb.jtraj(q0,q1,steps)
    traj14 = rtb.jtraj(q1,q0,steps)
    traj15 = rtb.jtraj(q0,q6,steps)
    traj16 = rtb.jtraj(q6,q7,steps)
    traj17 = rtb.jtraj(q7,q6,steps)
    traj18 = rtb.jtraj(q6,q8,steps)


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

def weld():
    a1 = 20
    a2 = 30
    a3 = 10

    # Define the articulated robot using DH parameters
    Articulated = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
],name='Articulated')

    ## Path and Trajectory Planning

    ## q Planned Paths
    # for Articulated Jonit Variables = ([x,y,z])

    q0 = SE3([30,
             -30,
             10]) #start
    
    q1 = SE3([30,
             0,
             10])
    
    q2 = SE3([30,
             30,
             10])
    
    q3 = SE3([0,
             30,
             10])
    
    q4 = SE3([-30,
             30,
             10])
    
    q5 = SE3([-30,
             0,
             10])
    
    q6 = SE3([-30,
             -30,
             10])
    
    q7 = SE3([0,
             -30,
             10])
    
    q8 = SE3([30,
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
    Articulated.plot(traj8, limits=[x1, x2, y1, y2, z1, z2], block=True)


#Link lengths and Joint Variable Frame
FI=LabelFrame(gggui,text='Link Lengths and Joint Variables',padx=15,pady=15,relief="solid",font=('Times New Roman', 12,'bold'),bd=3,bg='Lightblue')
FI.grid(row=0,column=0,padx=10, pady=5)

#Link Lenghts Label
a1 = Label(FI,text=('a1='),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
a1_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
mm1 =  Label(FI,text=('mm'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

a2 = Label(FI,text=('a2='),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
a2_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
mm2 =  Label(FI,text=('mm'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

a3 = Label(FI,text=('a3='),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
a3_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
mm3 =  Label(FI,text=('mm'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

a1.grid(row=0,column=0,padx=5)
a1_E.grid(row=0,column=1)
mm1.grid(row=0,column=2,)

a2.grid(row=1,column=0,padx=5)
a2_E.grid(row=1,column=1)
mm2.grid(row=1,column=2)

a3.grid(row=2,column=0,padx=5)
a3_E.grid(row=2,column=1)
mm3.grid(row=2,column=2)

#Joint Variable Label
T1 = Label(FI,text=('T1 = '),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
T1_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
deg1 = Label(FI,text=('deg'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

T2 = Label(FI,text=('T2 = '),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
T2_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
deg2 = Label(FI,text=('deg'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

T3 = Label(FI,text=('T3 = '),padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
T3_E = Entry(FI,width=8,font=('Times New Roman', 12),bg='#E9C46A')
deg3 = Label(FI,text=('deg'),font=('Times New Roman', 12,'bold'),bg='#2A9D8F')

T1.grid(row=0,column=3,padx=5)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3,padx=5)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

T3.grid(row=2,column=3,padx=5)
T3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

#Buttons Frame
BF = LabelFrame(gggui,text='Forward Kinematics',padx=10,pady=10,relief="solid",font=('Times New Roman',12,'bold'),bd=3,bg='Lightblue')
BF.grid(row=1,column=0,padx=10,pady=5)

#Buttons
FK = Button(BF,text='Forward',padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='blue',fg='white',command=f_k)
rst = Button(BF,text='Reset',padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='red',fg='white',command=reset)
IK = Button(BF,text='Inverse',padx=5,pady=5,relief="solid",font=('Times New Roman', 12,'bold'),bg='green',fg='white',command=i_k)

FK.grid(row=0,column=0,padx=5)
rst.grid(row=0,column=1,padx=5)
IK.grid(row=0,column=2,padx=5)

#Position Vectors Frame
PV = LabelFrame(gggui,text='Position Vector',padx=10,pady=10,relief="solid",font=('Times New Roman',12,'bold'),bd=3,bg='Lightblue')
PV.grid(row=2,column=0,padx=10, pady=5)

#Position Vector Label
X = Label(PV,text=('X = '),padx=0,pady=0,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
X_E = Entry(PV,width=10,font=('Times New Roman', 12),bg='#E9C46A')
mm4 =  Label(PV,text=('mm'),padx=0,pady=0,relief="solid",font=('Times New Roman', 12),bg='#2A9D8F')

Y = Label(PV,text=('Y = '),padx=0,pady=0,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
Y_E = Entry(PV,width=10,font=('Times New Roman', 12),bg='#E9C46A')
mm5 =  Label(PV,text=('mm'),padx=0,pady=0,relief="solid",font=('Times New Roman', 12),bg='#2A9D8F')

Z = Label(PV,text=('Z = '),padx=0,pady=0,relief="solid",font=('Times New Roman', 12,'bold'),bg='#2A9D8F')
Z_E = Entry(PV,width=10,font=('Times New Roman', 12),bg='#E9C46A')
mm6 =  Label(PV,text=('mm'),padx=0,pady=0,relief="solid",font=('Times New Roman', 12),bg='#2A9D8F')

X.grid(row=0,column=0,padx=5)
X_E.grid(row=0,column=1)
mm4.grid(row=0,column=2)

Y.grid(row=1,column=0,padx=5)
Y_E.grid(row=1,column=1)
mm5.grid(row=1,column=2)

Z.grid(row=2,column=0,padx=5)
Z_E.grid(row=2,column=1)
mm6.grid(row=2,column=2)

#insert image
img = PhotoImage(file="articulated.png")
img = img.subsample(3,3)
PI = Label(gggui,image=img)
PI.grid(row=3,column=0,columnspan=5,padx=10,pady=5,sticky=NSEW)

#Path and Trajectory Frame
PT = LabelFrame(gggui,text='Path and Trajectory',padx=10,pady=10,relief="solid",font=('Times New Roman',12,'bold'),bd=3,bg='Lightblue')
PT.grid(row=4,column=0,padx=10, pady=5)

PP = Button(PT,text="Pick and Place",font=('Times New Roman',12),bg='Gold',command=p_p)
Weld = Button(PT,text="Welding",font=('Times New Roman',12),bg='Silver',command=weld)

PP.grid(row=0,column=0,padx=(40,0))
Weld.grid(row=0,column=1,padx=(0,40))

  
gggui.mainloop()

 
