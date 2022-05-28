%%
%***************************************
% Suspension system
% -----------------
% Feedback control law + estimator
% Design via pole placement
%**************************************

clc
clear all


%% Parameters


k1 = 400; % Tyre stiffness constant
k2 = 600; % Spring stiffness constant
b =  550; % Damping constant of the suspension system
m1 = 28;  % Tyre and axle mass (unsprung)
m2 = 250; % 1/4 car body mass (sprung)


%******************
% System matrices
%******************

disp('System matrices:')


A = [      0            1           0           0   
     (-k1-k2)/m1     -b/m1       k2/m1        b/m1  
          0            0           0           1    
         k2/m2        b/m2      -k2/m2     -b/m2    ]
         

B = [ 0  
    -1/m1 
     0   
     1/m2]
 
B1 = [0
      k1/m1
      0
      0]

    
C = [0  0  1  0]
    
D = [0]

sys=ss(A,[B B1],C,[D 0]);
set(sys,'Inputname',{'F(t)','d(t)'})
set(sys,'Outputname',{'position of m_2 - chassis'})
impulse(sys)
%%
%************************
%Checking the stability
%************************
disp('Poles:')
eig (A)

%return
%%
%****************************************************
% Checking whether the system is controlable or not.
%****************************************************
CO = ctrb(A,B);
disp('Rank of the controllability matrix:');
rank(CO)

%%
%*************************************************
%Checking whether the system is observable or not
%*************************************************

Ob = obsv(A,C);
disp('Rank of the observability matrix');
rank(Ob)

%return
%% Controller design
%  Control law: u(t)= -K*x(t)

%******************************************
% Desire location of the closed-loop poles
%******************************************

%damping ratio
dr =  0.8; 
% Settling time: 7 seconds
ts = 7;

wn = 4.6/(dr*ts);
alpha = -dr*wn;
beta = wn*sqrt(1-dr^2);


%cl_poles= [alpha+beta*i alpha-beta*i 10*alpha 10*alpha]
cl_poles= [alpha+beta*i alpha-beta*i 10*alpha 10*alpha]

%**********************
%  Ackermann's method 
%**********************

disp('State feedback gain K computed via the Ackermann''s method:')
K=acker(A,B,cl_poles)

%return
%% Estimator design


%poles of the estimator
poles_estimator = 5 * cl_poles

%  factor =5;
%  dom = [factor*alpha factor*alpha];
%  poles_estimator = [dom 10*factor*alpha 10.2*factor*alpha]


% Computing the observer gain via the Ackermann's method
disp('Observer gain:')
L=acker(A',C',poles_estimator)'

% We get the same results if we use the place command
%L=place(A',C',poles_estimator)'

%%
%Initial conditions of the system
disp('Initial conditions:')
% x1 = 0.2 (mass m1), x2 = 0.1 (mass m2)
x0=[0.2 0 0.1 0]'




