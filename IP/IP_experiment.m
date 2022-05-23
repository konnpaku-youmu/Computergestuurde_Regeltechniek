clc;
clear;

rnd = randi(500, 1);

%% constants
Rm = 2.6;
Km = 0.00767;
Kb = 0.00767;
Kg = 3.7;
M = 0.455;
l = 0.305;
m = 0.210;
r = 0.635e-2;
g = 9.81;

f_c = 20;
w_c = 2*pi*f_c;
fs = 200;
T_s = 1/fs;

%% System model
A = [0        0                     1               0;
     0        0                     0               1;
     0     -(m*g)/M      -(Kg^2*Km*Kb)/(M*Rm*r^2)   0;
     0   (M+m)*g/(M*l)   (Kg^2*Km*Kb)/(M*Rm*r^2*l)  0];

B = [         0;
              0; 
       (Km*Kg)/(M*Rm*r);
      (-Km*Kg)/(r*Rm*M*l)];

C = [1 0 0 0;
     0 1 0 0];

D = [0;0];

OpenLoop = ss(A, B, C, D, T_s);

%% LQR
Q = [1.5   0    0   0;
      0    10   0   0;
      0    0    0   0;
      0    0    0   0];

R = 0.006;

%% response
pos_init = 0;
angle_init = 1*pi/180;
x0 = [pos_init  angle_init      0.0     0.0];
xd = [0.0    0.0];

[K, S, e] = lqrd(A, B, Q, R, T_s);

% Low pass filter & Backwards-Euler
LPF_A = [0              1           0               0;
             0  1/(1+w_c*T_s)           0               0;
             0              0           0               1;
             0              0           0   1/(1+w_c*T_s)];
LPF_B = [                    0                           0;
         (w_c*T_s)/(1+w_c*T_s)                           0;
                             0                           0;
                             0        (w_c*T_s)/(1+w_c*T_s)];
LPF_C = [-1/T_s    1/T_s         0         0;
              0        0    -1/T_s     1/T_s];
LPF_D = zeros(2);

