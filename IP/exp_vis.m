clc;
clear;

load('cart_state.mat');
load('setpoint.mat');
load('sys_state.mat');
load('v_in.mat');
% load("cart_state_sim_track.mat");

fs = 200;
sim_len = length(vin) / 200;
sim_sample_cnt = length(vin);

tout = linspace(0, sim_len, sim_sample_cnt);

output_names = ["$x$", "$\alpha$", "$\frac{dx}{dt}$", "$\frac{d\alpha}{dt}$"];
ylabels = ["Cart position (m)", "Rod angle (rad)"];

plot_start = 1;
plot_end = 28100;

for i=1:2
    fig(i) = figure(i);
    fig(i).Position = [200+720*(i-1), 200, 720, 560];

    plot(tout(plot_start:plot_end), cart_state(plot_start:plot_end, i), 'LineWidth', 1, 'DisplayName', output_names(i));
    hold on
%     plot(out.tout, out.state(:, i), 'LineWidth', 1, 'DisplayName', sprintf("Simulation: %s", output_names(i)));
    plot(tout(plot_start:plot_end), sp(plot_start:plot_end, i), 'LineWidth', 1.5, 'DisplayName', sprintf("Setpoint: %s", output_names(i)), 'Color', 'k');

    xlabel("Time(s)", 'Interpreter','latex');
    ylabel(ylabels(i), 'Interpreter','latex');

    title(sprintf("System response: %s", output_names(i)), 'Interpreter','latex');
    grid on
    hold on
    hl = legend('show');
    set(hl, 'Interpreter', 'latex');
end

fig3 = figure(3);
fig3.Position = [200+720*2, 200, 720, 560];
plot(tout(plot_start:plot_end), vin(plot_start:plot_end), 'LineWidth', 1);
xlabel("Time(s)", 'Interpreter','latex');
ylabel("Input voltage(V)", 'Interpreter','latex');
title("Input voltage", 'Interpreter','latex');
grid on;

% saveas(fig(1), "/home/yz/Projects/Reports/CACSD_IP/figures/exp_disturb_x.png", 'png');
% saveas(fig(2), "/home/yz/Projects/Reports/CACSD_IP/figures/exp_disturb_a.png", 'png');
% saveas(fig3, "/home/yz/Projects/Reports/CACSD_IP/figures/exp_disturb_v.png", 'png');