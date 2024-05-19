% CLear
clear
clc
close

disp('Articulated')

syms a1 a2 a3

%% Link Lengths
a1 = 20;
a2 = 30;
a3 = 10;

%% D-H Parameters [theta, d, r, alpha, offset]

H1 = Link([0,a1,0,pi/2,0]);
H1.qlim = [-pi/2 pi/2];

H2 = Link([0,0,a2,0,0]);
H2.qlim = [-pi/2 pi/2];

H3 = Link([0,0,a3,0,0]);
H3.qlim = [-pi/2 pi/2];


Arti1 = SerialLink([H1 H2 H3], 'name', 'Articulated');
Arti1.plot([0 0 0], 'workspace', [-75 75 -75 75 -50 75])
figure(1)
Arti1.teach

Af = ([pi/2,pi/2,pi/2]);
FK = Arti1.fkine(Af)

q_init=[0 0 0];
PV=transl([-15 30 35]);
IK = Arti1.ikine(PV,q_init,'mask',[1 1 1 0 0 0])

q_j1 = [pi/2 -pi/6 -pi/6]
J1 = jacob0(Arti1,q_j1)

q_j2 = [2*pi -pi/6 -pi/6]
J2 = jacob0(Arti1,q_j2)

%% Path and Trajetory
t = 0:0.5:2

%q paths
q0 = [0 0 0]; % start position
q1 = [0 -pi/6 -pi/6]; % pick position
q2 = [pi/2 0 0]; % revolve 90 degrees
q3 = [pi/2 -pi/6 -pi/6]; % place position1
q4 = [pi 0 0]; % revolve 180 degrees
q5 = [pi -pi/6 -pi/6]; % place position2
q6 = [3*pi/2 0 0];  %revolve 270 degrees
q7 = [3*pi/2 -pi/6 -pi/6]; % place position3
q8 = [2*pi 0 0]; %end position

% Trajectory
Traj1 = jtraj(q0,q1,t) %pick1 - start
Traj2 = jtraj(q1,q0,t) %up1
Traj3 = jtraj(q0,q2,t) %rotate1
Traj4 = jtraj(q2,q3,t) %place1
Traj5 = jtraj(q3,q2,t) %retract1
Traj6 = jtraj(q2,q0,t) %return1
Traj7 = jtraj(q0,q1,t) %pick2
Traj8 = jtraj(q1,q0,t) %up2
Traj9 = jtraj(q0,q4,t) %rotate2
Traj10 = jtraj(q4,q5,t) %place2
Traj11 = jtraj(q5,q4,t) %retract2
Traj12 = jtraj(q4,q0,t) %return2
Traj13 = jtraj(q0,q1,t) %pick3
Traj14 = jtraj(q1,q0,t) %up3
Traj15 = jtraj(q0,q6,t) %rotate3
Traj16 = jtraj(q6,q7,t) %place3
Traj17 = jtraj(q7,q6,t) %retract3
Traj18 = jtraj(q6,q8,t) %return3 - end 
figure(2)
plot(Arti1,Traj1)
plot(Arti1,Traj2)
plot(Arti1,Traj3)
plot(Arti1,Traj4)
plot(Arti1,Traj5)
plot(Arti1,Traj6)
plot(Arti1,Traj7)
plot(Arti1,Traj8)
plot(Arti1,Traj9)
plot(Arti1,Traj10)
plot(Arti1,Traj11)
plot(Arti1,Traj12)
plot(Arti1,Traj13)
plot(Arti1,Traj14)
plot(Arti1,Traj15)
plot(Arti1,Traj16)
plot(Arti1,Traj17)
plot(Arti1,Traj18)
