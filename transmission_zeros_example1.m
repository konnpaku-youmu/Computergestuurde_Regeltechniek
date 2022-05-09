%% Transmision Zeros
clc
clear all


%% State space model 

fprintf('------------------------------------\n');
fprintf('State space model in continuous-time\n')
fprintf('------------------------------------\n');
A = diag([-1 -6 -2 -8])
B=[ 1 0
    1 0
    0 1 
    0 1]
C=[1 0 1 0
   0 1 0 1]
D = zeros(2,2)

sys=ss(A,B,C,D);

%% Computing the transmission zeros
% Computing the transmision zeros via a generalized Eigenvalue problem
% A1 * v = lambda*A0*v
%
A1 =[A B; C D];
A0 =[eye(4,4) zeros(4,2)
     zeros(2,4) zeros(2,2)];

[V,Lam] = eig(A1,A0);  % A1*V = A0*V*Lam

fprintf('Transmission zeros:\n')
Lam = diag(Lam);
disp(Lam)
fprintf('Finite transmission zero:\n')
tz = Lam(3);
disp(tz)
fprintf('Initial state x0:\n')
x0 = V(1:4,3);
disp(x0)
fprintf('u0:\n')
u0 = V(5:end,3);
disp(u0)

%% Veryfing that the rank drops at s = transmission-zero

fprintf('-----------------------------\n');
fprintf('Transfer function matrix G(s)\n');
fprintf('-----------------------------\n');
G =tf(sys)

%Normal rank of G(s)
fprintf('--------------------\n');
fprintf('Normal rank of G(s)\n');
fprintf('--------------------\n');

fprintf('* G(s) at s = 1:\n');
Gtemp = evalfr(G,1);
disp(Gtemp)
fprintf('rank: %d\n',rank(Gtemp))

fprintf('\n* G(s) at s = 3+2i:\n');
Gtemp = evalfr(G,3+2i);
disp(Gtemp)
fprintf('rank: %d\n',rank(Gtemp))

fprintf('\n* G(s) at s = -5.2:\n');
Gtemp = evalfr(G,-5.2);
disp(Gtemp)
fprintf('rank: %d\n',rank(Gtemp))

fprintf('\n-------------------------------------\n');
fprintf('Rank of G(s) at the transmission zero\n');
fprintf('-------------------------------------\n');

fprintf('\n* G(s) at s = %g:\n',tz);
Gtemp = evalfr(G,tz);
disp(Gtemp)
fprintf('rank: %d\n',rank(Gtemp))

