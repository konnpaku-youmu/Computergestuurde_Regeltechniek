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

% %% Open loop analysis
% [~, eig_D] = eig(A);
% % Poles
% eig_D = complex(diag(eig_D)');
% % Transmission zeros
% tz = tzero(A, B, C, D);
% figure;
% plot(tz, 'o', 'MarkerSize', 9, 'DisplayName', 'Transmission zeros');
% hold on;
% plot(eig_D, 'x', 'Color', '#FF0000', 'LineWidth', 1, 'MarkerSize', 9, 'DisplayName', 'Poles');
% legend
% ytickformat('%1.1fj');
% xline(0, 'LineStyle','--');
% title("Transmission zeros & poles of the open loop system");
% 
% CO_open = ctrb(A, B);
% OB_open = obsv(A, C);
% 
% minsys = minreal(sys);

% subplot(121);
% pzmap(OpenLoop);
% title("Pole-zero map: open-loop");
% grid on

% full measurement
C = eye(4);
D = zeros(4, 1);

% LQR
Q = [ 1   0    0   0;
      0  0.25  0   0;
      0   0    0   0;
      0   0    0   0];

R = 0.01;

%% Closed loop

% % Closed loop analysis
% [eig_V, eig_D] = eig(A_close);
% % poles
% eig_D = complex(diag(eig_D)');
% % transmission zeros
% tz = tzero(A_close, B_close, C_close, D_close);
% figure;
% plot(tz, 'o', 'MarkerSize', 9, 'DisplayName', 'Transmission zeros');
% hold on;
% plot(eig_D, 'x', 'Color', '#FF0000', 'LineWidth', 1, 'MarkerSize', 9, 'DisplayName', 'Poles');
% legend
% ytickformat('%1.1fj');
% xline(0, 'LineStyle','--');
% title("Transmission zeros & poles of the open loop system");

% subplot(122);
% pzmap(ClosedLoop);
% title("Pole-zero map: closed-loop");
% grid on

% response
pos_init = 0;
angle_init = 15*pi/180;
x0 = [pos_init  angle_init  0   0]';
xd = [0.3          0.0      0   0]';
T_final = 5;

fig1 = figure(1);
fig2 = figure(2);

[K, S, e] = lqr(OpenLoop, Q, R);

output_names = ["$x$", "$\alpha$", "$\frac{dx}{dt}$", "$\frac{d\alpha}{dt}$"];
out = sim("closed_loop.slx");

for j =1:size(out.state, 1)
    set(0,'CurrentFigure',fig1);
    subplot(2,2,j);
    plot(out.tout, out.state(j, :), 'LineWidth', 1);
    title(sprintf("Closed-loop response: %s", output_names(j)), 'Interpreter','latex');
    grid on
    hold on 
end
set(0,'CurrentFigure',fig2);
plot(out.tout, out.vin, 'LineWidth', 1);
title("Input voltage (V)");
hold on

set(0,'CurrentFigure',fig2);
yline(5, 'DisplayName', 'Voltage limit', 'LineWidth', 1, 'LineStyle', '--');
legend







