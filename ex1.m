clc
clear

load("ex1_data.mat");

%% Ex1
sys = ss(A, B, C, D, 1e-3);

[reigA_v, reigA_d] = eig(A);
diag(reigA_d)'

% plot(eig_val, 'o');
% hold on

zz = tzero(A, B, C, D);
% plot(zz, 'x', 'DisplayName', 'Transmission zeros');
% hold on

zeta = zz(1);
M = [zeta*eye(length(A))-A -B; C D];
z = null(M);

x0 = z(1:length(A));
u0 = z(length(A)+1:end);

[num, den] = ss2tf(A, B, C, D, 1);

sys1_1 = tf(num(1, :), den);

z1_1 = zero(sys1_1);
p1_1 = pole(sys1_1);

% plot(z1_1, 'o', 'DisplayName', 'SISO 1-1 zeros');
% legend

%% Ex2

% rank: controlability
ctrl_mat = ctrb(A, B);
rank_c = rank(ctrl_mat);

% PBH: ctrl
[leigA_v, leigA_d] = eig(A');

B' * leigA_v

% rank: observability
obsv_mat = obsv(A, C);
rank_o = rank(obsv_mat);

% PBH: obsv
C * reigA_v

minsys = minreal(sys);
[mineig_v, mineig_d] = eig(minsys.A);

diag(mineig_d)'


