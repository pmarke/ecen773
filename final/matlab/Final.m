%%Question 1:

% System 1 and 2
A = [0 1; 1 0];
B1 = [0;-2];
B2 = [0;2];
C = [1 0];

% Controllability test
rank(ctrb(A,B1))
rank(ctrb(A,B2))

% Observability testU
rank(obsv(A,C))
