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

% C = zeros(6, 12);
% C(1:3, 1:3) = eye(3);
% C(4:6, 7:9) = eye(3);
% 
% D = zeros(6, 4);

C = eye(12);
D = zeros(12, 4);

quad_ss = ss(A, B, C, D);

u_eq = [m*g/(4*k*c_m), m*g/(4*k*c_m), m*g/(4*k*c_m), m*g/(4*k*c_m)];

% sim("linear_test_quadcopter.slx");
% 
% output_names = ["$x$", "$y$", "$z$", "$\phi$", "$\theta$", "$\psi$"];
% ylabels = ["$x (m)$", "$y (m)$", "$z (m)$", "$Roll (rad)$", "$Pitch (rad)$", "$Yaw (rad)$"];
% 
% for i=1:size(lin_out, 2)
%     subplot(2,3,i)
%     
%     plot(tout, lin_out(:, i), 'DisplayName', "SS model", 'LineWidth', 1);
%     hold on
%     plot(tout, quad_out(:, i), 'DisplayName', "Non-linear model", 'LineWidth', 1);
%     xlabel("Time(s)", 'Interpreter','latex');
%     ylabel(ylabels(i), 'Interpreter','latex');
%     title(sprintf("Output: %s", output_names(i)), 'Interpreter', 'latex');
% 
%     hl = legend('show');
%     set(hl, 'Interpreter', 'latex');
% 
%     grid on
%     
% end

T_s = 0.05;
quad_ss_d = c2d(quad_ss, T_s, 'tustin');
A_z = quad_ss_d.A;
B_z = quad_ss_d.B;
C_z = quad_ss_d.C;
D_z = quad_ss_d.D;

CO = ctrb(A_z, B_z);
rank_CO = rank(CO);

OB = obsv(A_z, C_z);
rank_OB = rank(OB);

% pzmap(quad_ss_d);

%% Full-state feedback
ref_A = [A_z - eye(12), B_z; C_z, D_z];
ref_b = [zeros(12); eye(12)];
N = ref_A \ ref_b;
N_x = N(1:12, :);
N_u = N(13:16, :);

Q = diag([10, 10, 50, 2, 2, 5, 5, 5, 5, 1, 1, 1]);
R = diag([0.005, 0.005, 0.005, 0.005]);

[K, S, e] = dlqr(A_z, B_z, Q, R);

sim("LQR_sim.slx", Tmax);
% generate_report(0);

output_names = ["$x$", "$y$", "$z$", "$\phi$", "$\theta$", "$\psi$"];
ylabels = ["$x (m)$", "$y (m)$", "$z (m)$", "$Roll (rad)$", "$Pitch (rad)$", "$Yaw (rad)$"];

for i=1:6
    subplot(2,3,i)
    if(i<=3)
        plot(states_quadcopter.time, simout(:, i), 'DisplayName', "Reference", 'LineWidth', 1);
        hold on
    end
    plot(states_quadcopter.time, simout(:, i+3), 'DisplayName', "No payload", 'LineWidth', 1);
    xlabel("Time(s)", 'Interpreter','latex');
    ylabel(ylabels(i), 'Interpreter','latex');
    title(sprintf("Output: %s", output_names(i)), 'Interpreter', 'latex');

    hl = legend('show');
    set(hl, 'Interpreter', 'latex');

    grid on
end
