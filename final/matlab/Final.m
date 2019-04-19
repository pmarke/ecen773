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

%% Question 3
%%% a)
A = [-2 4; 1 1];
b = [1;1];
C = [1 0];
eig(A);

syms s
C*inv(s*eye(2)-A)*B;

%%% b)
syms t to
PHI = [1 0; 1/2*(t^2-to^2) 1];
A = diff(PHI,t)*inv(PHI)


