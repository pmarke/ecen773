%% Problem 1
% x_dot = Ax + Bu
% y = Cx + Du
A = [1 0; 4 -1];
B = [-1;2];
C = [3 0];
D = 5;

%% a)

%% b)
[V,J] = jordan(A);
syms t;
V*expm(J*t)*inv(V);

%% c)
A^2 - eye(2)

%% d)

syms s;
C*inv(s*eye(2)-A)*B + D