A = [-4 -2; 0 1];
B = [1;0];
C = [-2 1];
D = 1;

syms s l1 l2 

P = [s*eye(2) - A, B; -C D]

[V,J] = jordan(A);

X_t = V*[l1, 0; 0 l2]*inv(V)