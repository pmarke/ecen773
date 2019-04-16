%% 5)

A = [2 0 0 ; 0 -1 0; 0 0 -1];
B = [1 0 ; 1 0; 0 1];
C = [1 0 2; 0 -1 0];
D = [1 0; 1 0];

sys = ss(A,B,C,D);

%%% a)
% Whare are the poles of the system? What is the multiplicity of each pole?

syms s

G_s = C*inv(s*eye(3)-A)*B +D;
% Poles of the system are 2,-1,-1

%%% b)
% What are the invariant zeros of the system(finite and infinite)?
P = [s*eye(3)-A, B; -C D]

% An invariant zero will cause P to lose rank.
rank_2 = rank(subs(P,2))
rank_1 = rank(subs(P,-1))
rank_0 = rank(subs(P,0))

% Invariant zeros are 0, inf and 2

%%% c) Transmission zeros are the invariant zeros that are not eigenvalues
%%% of A.

% Since 2 is an eigen value of A, 0 is the only transmission zero.

%% 6)
clear all;
A = [-1 0 0; 0 -2 0; 0 0 -2];
B = [2 -2; -2 4; -4 2];
C = [1 1 0; 1 0 1];
D = [0 0; 0 0];
sys = ss(A,B,C,D);

%%% a) Use the transmission zero to fine an input u(t) and the initial
%%% condition x(0) that will result in y(t) = 0 for all time.

lambda_A = eig(A);      % Eigen values are -2, -2, -1
inv_zeros = tzero(sys); % Invariant zeros are 1
                        % The transmission zero must be 1 since 1 is not an
                        % eigen value of A.
                        
% Now that we have a transmission zero, we can find an x(s) and u(s) that
% are in the null space of P(s=transmission_zero).
syms s
P = [s*eye(3)-A, B; -C D];
Pz = [1*eye(3) - A,B; -C D];
nu = null(double(subs(P,1)));
nu = null(Pz)

% Thus the solution is x(0) = -[2 -2 -2]' and u(t) = [-1 1]'exp(t)'

%%% b) Verify your solution by calculating the output for the given input

xo = nu(1:3);
uo = -nu(4:5);

% opts = odeset('RelTol',1e-2,'AbsTol',1e-8);

[t,x] = ode45(@(t,x) prob6(t,x,A,B,C,D,uo), [0 2],xo);

y = C*x';

sum(y,2)

syms t
u = uo.*exp(0:0.01:2);

[y,x] = lsim(ss(A,B,C,0),uo.*exp(0:0.01:5),0:0.01:5,xo);
sum(y)



function  dx = prob6(t,x,A,B,C,D,uo)

u = uo*exp(t);

dx = A*x+B*u;


end


                        



