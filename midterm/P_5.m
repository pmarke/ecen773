%%Problem 5


% Verify transfer function
w = 5
H1 = tf([1],[1 0 w^2]);
H2 = tf([1],[1 0]);
H3 = tf([1 0],[1 0 w^2]);

y = (H2+H3)*H1;

% Select the system matrices
syms w s

A = [0 -2*w^2 0 -w^4 0;...
     1    0   0   0  0;...
     0    1   0   0  0;...
     0    0   1   0  0;...
     0    0   0   1  0];

B = [1;0;0;0;0];
C = [0 0 2 0 w^2];
D = 0;

% Verify that the selected system gives the same transfer function;
G = C*inv(s*eye(5)-A)*B;
G = simplify(G);

% Compute the Jordan normal form of the matrix A
[V,J] = jordan(A)