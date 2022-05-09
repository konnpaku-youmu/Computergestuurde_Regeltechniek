% First Example of LQR

clear all
close all
clc

%% Dynamical system

% x(k+1) =  A*x(k) + B*u(k)
%   y(k) =  C*x(k)

A=1;
B=1;
C=1;
D=0;
Ts =1  ; %Sampling time

sys =ss(A,B,C,D,Ts);

%% LQR design

% Control Law: u(k) = -K*x(k)
% Weighting matrices:
% Q = 1, R 

Q =1;

%Different R gains
R_vector = [0.05 1  10   20 ]

%Computing the state-feedback gains
nGains =length(R_vector);
K_vector =zeros(nGains,1);
lab ={};
for j=1:nGains
    K_vector (j) = dlqr(A,B,Q,R_vector(j));
    lab{j}=sprintf('R=%g',R_vector(j));
end    

%% Plotting 

figure;
x0 =10; %initial condition
Tfinal =20;
for j=1:nGains
    %close loop system: 
    %            x(k+1) = (A-BK)x(k) 
    %              y(k) =  C*x(k)
    K = K_vector(j);
    sys_cl = ss(A-B*K,0,C,D,Ts);
    [y,t,x]=initial(sys_cl,x0,Tfinal);
    u = -K*x;
    subplot(1,2,1); hold on
    plot(t,x,'.-');
    xlabel('k');ylabel('x(k)');box on
    subplot(1,2,2); hold on
    plot(t,u,'.-');
    xlabel('k');ylabel('u(k)');box on
end
subplot(1,2,1);
legend(lab)
subplot(1,2,2);
legend(lab)

%% 

R_vector2=[0.001:1:250];
%R_vector2=[0.001:10:1000];
nGains2 = length(R_vector2);
K_vector2 = zeros(nGains2,1);
for j=1:nGains2
    K_vector2 (j) = dlqr(A,B,Q,R_vector2(j));
end    

figure;
plot(R_vector2,K_vector2,'.-')
ylabel('state feedback gain - K')
xlabel('R')



