%% Question 6

A = [-0.038   18.984   0      -32.174;...
     -0.001  -0.632    1       0;...
      0       -0.759   -0.518   0;...
      0        0        1       0];

B =   [10.1       0;...
        0        -0.0086;...
        0.025    -0.011;...
        0         0];
  
be = B(:,2);
% a) Show that the longitudinal dynamics are completely controllable with
% just an elevator.

rank(ctrb(A,B(:,2)))

% b) Can you control the system from xo = [250 0 0 0]' to x60 = [300 10 2
% 5]' with only a constant elevator input?
% x60 = expm(A60)xo + integral expm(At)*be*delta_e dt

xd = [300 10*pi/180 2*pi/180 5*pi/180]';   % Desired state after 60 seconds
xo = [250 0 0 0]';    % Initial state at trim
z = xd - expm(A*60)*xo;
y = inv(A)*(expm(A*60)-eye(4))*B(:,2);

% c) Find a time varying delta_e to control xo to xd

% Construct the grammian
% syms t

temp = @(x) expm(A*x)*be*be'*expm(A'*x);
Wr_d = integral(temp,0,60,'ArrayValued',true)

% If xd - expm(A*60)*xo is in the range of the grammian Wr_d such
% that xd-expm(A*60)*xo = Wr_d*zeta then we can
% find a control law delta_e(t) = be'*expm(A'(60-t))*zeta that transforms
% xo to xd in sixy seconds. 

% Find zeta
zeta = inv(Wr_d)*(xd - expm(A*60)*xo);



% d) Simulate your results from part c to verify you achieved the desired
% end state after 60 seconds.

[t,x] = ode45(@(t,x) aircraftDynamics(t,x,A,be,zeta),[0 60],xo);

error = norm(xd - x(end,:)')

%%

figure(1),clf;
subplot(2,2,1)
plot(t,x(:,1));
hold on
plot(t,xd(1)*ones(length(t),1));
xlabel("time (s)")
ylabel("velocity (ft/s)")
title("Airspeed")
legend("airspeed(t)","Desired airspeed")

subplot(2,2,2)
plot(t,x(:,2));
hold on
plot(t,xd(2)*ones(length(t),1));
xlabel("time (s)")
ylabel("angle of attack (rad)")
title("Angle of Attack")
legend("Angle of Attack(t)","Desired Angle of Attack")

subplot(2,2,3)
plot(t,x(:,3));
hold on
plot(t,xd(3)*ones(length(t),1));
xlabel("time (s)")
ylabel("Pitch Rate (rad/s)")
title("Pitch Rate")
legend("Pitch Rate(t)","Pitch Rate")

subplot(2,2,4)
plot(t,x(:,4));
hold on
plot(t,xd(4)*ones(length(t),1));
xlabel("time (s)")
ylabel("Pitch Angle (rad)")
title("Pitch Angle")
legend("Pitch Angle(t)","Pitch Angle")

%% e)

A*xd

%% Question 7

% Find a state feedback control u = -kx that puts the poles 15x further to
% the left i nthe complex plane. Pick poles that will maintain the original
% ratio.

lambda = eig(A);    % Get the original eigen values of A


% Form the characteristic polynomial for both modes
% S^2 + 2*eta*wn*s + wn^2    
a1 = -lambda(1)-lambda(2);
a2 = -lambda(1)*-lambda(2);
wn1 = sqrt(a2);
eta1 = a1/wn1/2;

b1 = -lambda(3)-lambda(4);
b2 = -lambda(3)*-lambda(4);
wn2 = sqrt(b2);
eta2 = b2/wn2/2;

p1 = -eta1*wn1*15 + wn1*sqrt(1-eta1^2);
p2 = -eta1*wn1*15 - wn1*sqrt(1-eta1^2);

p3 = -eta2*wn2*15 + wn2*sqrt(1-eta2^2);
p4 = -eta2*wn2*15 - wn2*sqrt(1-eta2^2);

K = place(A,B,[p1,p2,p3,p4]);

SYS_OL = ss(A,be,eye(4),0);
SYS_cl = ss(A-B*K,be,eye(4),0);
figure(2)
step(SYS_OL)
hold on
step(SYS_cl)
