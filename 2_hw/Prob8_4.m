%% Problem 8_4


%%% a) Compute the state transition matrix phi(t,t0)

syms t t0


PHI1 = [exp(t)*cos(2*(t)) exp(-2*(t))*sin(2*(t));...
       -exp(t)*sin(2*(t)) exp(-2*(t))*cos(2*(t))];
   
PHI2 = [exp(t0)*cos(2*(t0)) exp(-2*(t0))*sin(2*(t0));...
       -exp(t0)*sin(2*(t0)) exp(-2*(t0))*cos(2*(t0))];

% Property 5.3 states
% PHI(t,s)PHI(s,tau) = PHI(t,tau)

% Property 5.4 states
% PHI(t,tau)^-1 = PHI(tau,t)


PHI = PHI1*inv(PHI2);
PHI = simplify(PHI)

%%% b) Compute the matrix A(t) that corresponds to the given state transition matrix

% Phi_dot = A(t)*PHI
PHI_dot = diff(PHI,t);

A = PHI_dot*inv(PHI);
A = simplify(A)

%%% c) Compute the eigen values of A(t)
lambda = eig(A);
lambda = simplify(lambda)

%%% d) Classify this system in terms of Lyapunov stability
% We can look at the stability of the sytem by analyzing the
% the state transitioin matrix to see if it become arbitrarily large.
% The sin and cos functions are bounded between [-1,1], and the exponential
% functions are bounded between [0,0) depending on the value of t. Thus
% we can analyze the stability of PHI by looking at is as t -> infinity. 
% By inspection, we
% can see that ||PHI|| -> inf and t -> inf. Thus the system is not stable.

parmas.m


   

   
