clc;
clear;

load("references_01.mat");

%% Constants
Ts = 0.05;
m = 0.5;
L = 0.25;
k = 3e-6;
b = 1e-7;
g = 9.81;
k_d = 0.25;
I_xx = 5e-3;
I_yy = 5e-3;
I_zz = 1e-2;
c_m = 1e4;

A = zeros(12);
A(1:3, 4:6) = eye(3); % x_dot = v_x
A(4:6, 4:6) = (-k_d/m) * eye(3);
A(7:9, 10:12) = eye(3);

B = zeros(12, 4);
% v_z
B(6, :) = (k*c_m) / m;
% w_x
B(10, 1) = (L*k*c_m) / I_xx;
B(10, 3) = -B(10, 1);
% w_y
B(11, 2) = (L*k*c_m) / I_yy;
B(11, 4) = -B(11, 2);
% w_z
B(12, 1) = (b*c_m) / I_zz;
B(12, 2) = -B(12, 1);
B(12, 3) = B(12, 1);
B(12, 4) = -B(12, 1);
