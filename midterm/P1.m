%% Problem 1
% x_dot = Ax + Bu
% y = Cx + Du
A = [1 0; 4 -1];
B = [-1;2];
C = [3 0];
D = 5;

%% a)

%% b)  Eigen value decomposition
[V,J] = jordan(A);
% Compute the state stransition matrix
syms t;
V*expm(J*t)*inv(V);

%% c) Cayley-Hamilton
A^2 - eye(2)

%% d) Find the state transfer function G(s)
% G(s) = C*inv(SI-A)*B + D
syms s;
C*inv(s*eye(2)-A)*B + D