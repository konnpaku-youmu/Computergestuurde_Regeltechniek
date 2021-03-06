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
A(4, 8) = g;
A(5, 7) = -g;
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

C = zeros(6, 12);
C(1:3, 1:3) = eye(3);
C(4:6, 7:9) = eye(3);

D = zeros(6, 4);

quad_ss = ss(A, B, C, D);

u_eq = [m*g/(4*k*c_m), m*g/(4*k*c_m), m*g/(4*k*c_m), m*g/(4*k*c_m)];

T_s = 0.05;
quad_ss_d = c2d(quad_ss, T_s, 'tustin');
A_z = quad_ss_d.A;
B_z = quad_ss_d.B;
C_z = quad_ss_d.C;
D_z = quad_ss_d.D;

% integral: x,y,z,phi,theta,psi
A_i = [eye(3), C_z(1:3, :); zeros(12, 3), A_z];
B_i = [D_z(1:3, :); B_z];

rank(ctrb(A_z, B_z))
rank(obsv(A_z, C_z))

Q = diag([1, 1, 2.5, 75, 75, 125, 75, 75, 90, 5, 5, 5, 50, 50, 100]);
R = diag([0.005, 0.005, 0.005, 0.005]);

[K, S, e] = dlqr(A_i, B_i, Q, R);
K_i = K(:, 1:3);
K_s = K(:, 4:end);

Poles_sys = eig(A_z-B_z*K_s);

Q_k= 1e-4*eye(12);
R_k = diag([2.5e-5, 2.5e-5, 2.5e-5, 7.57e-5, 7.57e-5, 7.57e-5]);

[M, P, Cov, Poles_esti] = dlqe(A_z, eye(12), C_z, Q_k, R_k);
L_k = A_z*M;

sim("LQG_sim.slx");

fig = figure(1);
fig.Position = [200, 200, 1200, 800];

output_names = ["$x$", "$y$", "$z$", "$\phi$", "$\theta$", "$\psi$"];
ylabels = ["$x (m)$", "$y (m)$", "$z (m)$", "$Roll (rad)$", "$Pitch (rad)$", "$Yaw (rad)$"];

for i=1:3
    subplot(3,1,i);

    if(i<=3)
        plot(tout, simout(:, i), 'DisplayName', "Reference", 'LineWidth', 1);
        hold on
    end

    plot(tout, simout(:, i+3), 'DisplayName', "LQG: Payload = 0.1kg", 'LineWidth', 1);
    xlabel("Time(s)", 'Interpreter','latex');
    ylabel(ylabels(i), 'Interpreter','latex');
    title(sprintf("Output: %s", output_names(i)), 'Interpreter', 'latex');

    hl = legend('show');
    set(hl, 'Interpreter', 'latex');

    grid on
end

