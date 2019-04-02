%% Question 6

zeta = 2;
wn = 1.5;

% System
A = [0 1;...
     -2*zeta*wn wn^2]; 
C = [1 0];
B = [0 1]';

% Initial
x_hato = [0;0];
xo = [1;2];
% Error
e_15 = [0.1 0.1]';  % Error at 15 seconds
eo = x_hato - xo;      % Initial error

%%% a)

syms l1 l2

eqn = e_15 == expm((A-[l1,l2]'*C)*15)*eo;

[L1,L2] = vpasolve(eqn,[l1,l2]);
L = double([L1;L2]);

% Test
expm((A-L*C)*15)*eo;

%%% b)
% Using the observer gain L designed in part A. Plot how the estimated
% states track the true states.

zo = [xo;x_hato];

[t,z] = ode45(@(t,z) systemDynamics(t,z,A,B,C,L,1),[0 5],zo);
[t2,z2] = ode45(@(t,z) systemDynamics(t,z,A,B,C,L,0),[0 5],zo);
%%

figure(1)
clf;
subplot(2,2,1);
plot(t,z(:,1),'*')
hold on
plot(t,z(:,3))
plot(t,z(:,1) - z(:,3),'r')
title('z1 with input')
legend('True','Estimated','Error');
xlabel('t')

subplot(2,2,2);
plot(t,z(:,2),'*')
hold on
plot(t,z(:,4))
plot(t,z(:,2) - z(:,4),'r')
title('z2 with input')
legend('True','Estimated','Error');
xlabel('t')

subplot(2,2,3);
plot(t2,z2(:,1),'*')
hold on
plot(t2,z2(:,3))
plot(t2,z2(:,1) - z2(:,3),'r')
title('z1 without input')
legend('True','Estimated','Error');
xlabel('t')

subplot(2,2,4);
plot(t2,z2(:,2),'*')
hold on
plot(t2,z2(:,4))
plot(t2,z2(:,2) - z2(:,4),'r')
title('z2 without input')
legend('True','Estimated','Error');
xlabel('t')

%%

function dxdt = systemDynamics(t,x,A,B,C,L,m)

    z = x(1:2);              % System states    
    z_hat = x(3:4);          % Estimated system states
    
    u = (3 + 0.5*sin(0.75*t));        % Input
    z_dot = A*(z) + B*u*m;        % System dynamics
    
    z_hat_dot = A*(z_hat) + B*u - L*C*(z_hat - z);
    
    dxdt = [z_dot;z_hat_dot];

end