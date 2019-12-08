syms t1 t2 a1 a2 real
ptab_2R = [t1 0 a1 0;t2 0 a2 0];

r_2R = Robot_4(2,ptab_2R,'RR')
syms l1 l2 real
PL = [l1*cos(t1) a1*cos(t1)+l2*cos(t1+t2);l1*sin(t1) a1*sin(t1)+l2*sin(t1+t2);0 0];
PM = [0 a1*cos(t1);0 a1*sin(t1); 0 0];
Zcf_b = [0 0;0 0;1 1];
syms kr1 kr2 ml1 ml2 mm1 mm2 Il1 Il2 Im1 Im2 g real
GR = [kr1 kr2];
ML = [ml1 ml2];
MM = [mm1 mm2];
IL = [Il1 Il2];
IM = [Im1 Im2];
g0 = [0 -g 0]';
syms dt1 dt2
[B,C,g_] = Manipulator_Dynamics_2(r_2R.n,r_2R.Joint_C,PL,PM,Zcf_b,r_2R.link_type,GR,ML,MM,IL,IM,g0,[dt1 dt2]);

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