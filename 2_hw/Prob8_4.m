%% Problem 8_4


% b) Compute the matrix A(t) that corresponds to the given state transition
% matrix
syms t t0

PHI = [exp(t-t0)*cos(2*(t-t0)) exp(-2*(t-t0))*sin(2*(t-t0));...
       -exp(t-t0)*sin(2*(t-t0)) exp(-2*(t-t0))*cos(2*(t-t0))];

% Phi_dot = A(t)*PHI
PHI_dot = diff(PHI,t);

A = PHI_dot*inv(PHI);
simplify(A)

% c) Compute the eigen values of A(t)
lambda = eig(A)


   
PHI1 = [exp(t)*cos(2*(t)) exp(-2*(t))*sin(2*(t));...
       -exp(t)*sin(2*(t)) exp(-2*(t))*cos(2*(t))];
   
PHI2 = [exp(t0)*cos(2*(t0)) exp(-2*(t0))*sin(2*(t0));...
       -exp(t0)*sin(2*(t0)) exp(-2*(t0))*cos(2*(t0))];

% Property 5.3 states
% PHI(t,s)PHI(s,tau) = PHI(t,tau)

% Property 5.4 states
% PHI(t,tau)^-1 = PHI(tau,t)
   
temp = PHI1*inv(PHI2);
simplify(temp)