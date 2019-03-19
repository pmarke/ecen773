%% Quiz 9

% Determine if the system is stabilizable and/or detectable

A = [0 -1 1;...
     1 -2 1;...
     0  1 -1];
 
B = [ 1 0;...
     1 1;...
     1 2];
 
C = [0 1 0];

% For the system to be stabilizable, none of the eigen vectors
% corresponding with non negative eigen vectors can be in the nullspace of
% B. 

eig(A);
% Has eigen values -2,-1,0.
% We need to make sure that the eigen vector associated with the eigen
% value 0 is not in the null space of B.

vo = null(A - 0*eye(3))
B'*vo

C*vo

[V,D] = svd(A)