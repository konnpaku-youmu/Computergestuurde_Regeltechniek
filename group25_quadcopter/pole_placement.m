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

%% state feedback
ref_A = [A_z - eye(12), B_z; C_z, D_z];
ref_b = [zeros(12); eye(6, 12)];
N = ref_A \ ref_b;
N_x = N(1:12, :);
N_u = N(13:16, :);

dr = 1;
ts = 6;

wn = 4.6/(dr*ts);
alpha = -dr*wn;         
beta = wn*sqrt(1-dr^2);

poles_dominant = [alpha + 1j*beta, alpha - 1j*beta];
poles_nondom = 9*alpha*ones(1, 10)-(-5:4)*0.01;
poles = [poles_dominant, poles_nondom];

K = place(A_z,  B_z,  exp(poles * T_s));
L = place(A_z', C_z', exp(5*poles*T_s)).';

A_c = A_z - B_z*K - L*C_z + L*D_z*K;

pole_comp = eig(A_c);

sim("pole_placement_sim.slx");

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

    plot(tout, simout(:, i+3), 'DisplayName', "Pole-placement: Payload = 0.1kg", 'LineWidth', 1);
    xlabel("Time(s)", 'Interpreter','latex');
    ylabel(ylabels(i), 'Interpreter','latex');
    title(sprintf("Output: %s", output_names(i)), 'Interpreter', 'latex');

    hl = legend('show');
    set(hl, 'Interpreter', 'latex');

    grid on
end
