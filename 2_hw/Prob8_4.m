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


   
