syms d1 t2 l2
ptab_PR = [pi pi/2 0 d1;t2 0 l2 0];

r_PR = Robot_5(2,ptab_PR,'PR')
PL = [0 -l2*cos(t2);0 0;d1 d1+l2*sin(t2)];
PM = [0 0;0 0;0 d1];
Zcf_b = [0 0;0 1;1 0];
syms kr1 kr2 ml1 ml2 mm1 mm2 Il1 Il2 Im1 Im2 real
GR = [kr1 kr2];
ML = [ml1 ml2];
MM = [mm1 mm2];
IL = [Il1 Il2];
IM = [Im1 Im2];
syms g dd1 dt2
g0 = [-g 0 0]';
[B,C,g_] = Manipulator_Dynamics_2(r_PR.n,r_PR.Joint_C,PL,PM,Zcf_b,r_PR.link_type,GR,ML,MM,IL,IM,g0,[dd1 dt2]);

% syms t
% dd1 = 0.1;dt2 = 1;
% d1 = 0.1*t;t2 = pi*t;
a1 = 1;a2 = 1;
l1  =  0.5;l2  =  0.5;
kr1 = 100;kr2 = 100;
ml1  = 50;ml2  = 50;
mm1 = 5;mm2 = 5;
Il1 = 10;Il2 = Il1;
Im1 = 0.01;Im2 = 0.01;
g=9.81;

eval(B)
eval(C)
eval(g_)
