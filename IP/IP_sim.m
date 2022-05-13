clc;
clear;

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

f_c = 50;
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

OpenLoop = ss(A, B, C, D);

%% SS model: LPF and Backward-Euler

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

%% LQR
Q = [ 4   0    0   0;
      0   6    0   0;
      0   0    0   0;
      0   0    0   0];

R = 0.007;

%% response
pos_init = 0;
angle_init = 15*pi/180;
x0 = [pos_init  angle_init      0.0     0.0];
xd = [0.3    0.0];

[K, S, e] = lqr(OpenLoop, Q, R);

output_names = ["$x$", "$\alpha$", "$\frac{dx}{dt}$", "$\frac{d\alpha}{dt}$"];
out = sim("closed_loop2.slx");

fig1 = figure(1);
fig2 = figure(2);

for j =1:size(out.state, 2)
    set(0,'CurrentFigure',fig1);
    subplot(2,1,j);
    plot(out.tout, out.state(:, j), 'LineWidth', 1);
    title(sprintf("Closed-loop response: %s", output_names(j)), 'Interpreter','latex');
    grid on
    hold on 
end

set(0,'CurrentFigure',fig2);
plot(out.tout, out.vin, 'LineWidth', 1);
title("Input voltage (V)");
hold on
legend



