clc;
clear;
load("setpoint.mat");
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

f_c = [10];
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

%% LQR
Q = [ 1.5   0    0   0;
      0     10    0   0;
      0     0    0   0;
      0     0    0   0];

R = 0.006;

%% response
pos_init = -0.015;
angle_init = -8.5*pi/180;
x0 = [pos_init  angle_init      0.0     0.0];

xd = sp(1:12800, 1);

ip_d = c2d(OpenLoop, T_s, 'tustin');
A_z = ip_d.A;
B_z = ip_d.B;
C_z = ip_d.C;
D_z = ip_d.D;

[K, S, e] = dlqr(A_z, B_z, Q, R);

output_names = ["$x$", "$\alpha$", "$\frac{dx}{dt}$", "$\frac{d\alpha}{dt}$"];
ylabels = ["Cart position (m)", "Rod angle (rad)"];

fig1 = figure(1);
fig1.Position = [200, 200, 1600, 480];

% fig2 = figure(2);
% fig2.Position = [1640, 200, 760, 800];

for i=1:length(w_c)
    %% SS model: LPF and Backward-Euler
    LPF_A = [0              1           0               0;
             0  1/(1+w_c(i)*T_s)           0               0;
             0              0           0               1;
             0              0           0   1/(1+w_c(i)*T_s)];
    LPF_B = [                    0                           0;
             (w_c(i)*T_s)/(1+w_c(i)*T_s)                           0;
                                 0                           0;
                                 0        (w_c(i)*T_s)/(1+w_c(i)*T_s)];
    LPF_C = [-1/T_s    1/T_s         0         0;
                  0        0    -1/T_s     1/T_s];
    LPF_D = zeros(2);
    
    out = sim("closed_loop2.slx");

    label = sprintf("Deadzone = 0.6V");
    
    for j =1:size(out.state, 2)
        set(0,'CurrentFigure',fig1);
        subplot(1,2,j);
        
        plot(out.tout, out.sp(:, j), 'LineWidth', 1.5, 'DisplayName', "Setpoint");
        hold on
        plot(out.tout, out.state(:, j), 'LineWidth', 1, 'DisplayName', label);
        
        xlabel("Time(s)", 'Interpreter','latex');
        ylabel(ylabels(j), 'Interpreter','latex');
        title(sprintf("Closed-loop response: %s", output_names(j)), 'Interpreter','latex');
        grid on
        hold on
        hl = legend('show');
        set(hl, 'Interpreter', 'latex');
    end
    
%     set(0,'CurrentFigure',fig2);
%     stairs(out.tout, out.vin, 'LineWidth', 1, 'DisplayName', label);
%     title("Input voltage", 'Interpreter','latex');
%     xlabel("Time(s)", 'Interpreter','latex');
%     ylabel("Voltage (V)", 'Interpreter','latex');
%     grid on
%     hold on
%     legend
end

% saveas(fig1, "/home/yz/Projects/Reports/CACSD_IP/figures/closed2_sp.png", 'png');

