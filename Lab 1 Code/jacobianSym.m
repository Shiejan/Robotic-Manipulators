clear;

syms t1 t2 t3 t4 t5 t6
t = [t1, t2, t3, t4, t5, t6];
% DH = [alpha, a, d, theta]
L1 = sym('L1', 'real'); % 117.1
L2 = sym('L2', 'real'); % 172.6
L3 = sym('L3', 'real'); % 108.7
L4 = sym('L4', 'real'); % 74.4
DH = [0, 0, L1, t1; pi/2, 0, 0, t2; 0, L2, 0, t3; pi/2, 0, L3, t4; -pi/2, 0, 0, t5; pi/2, 0, L4, t6];
syms dt1 dt2 dt3 dt4 dt5 dt6 real
dt = [dt1, dt2, dt3, dt4, dt5, dt6];
N = 6;
[jacobian, determinant] = computeJacobian(DH,t,dt,N)