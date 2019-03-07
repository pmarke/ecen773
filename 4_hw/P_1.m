%% Problem 1

syms w

A = [ 0 1 0 0;... 
    3*w^2 0 0 2*w;... 
    0 0 0 1;... 
    0 -2*w 0 1];

B = [0 0;...
     1 0;...
     0 0;...
     0 1];
 
 % a) Show that the system is controllable from u
 
 % Controllability matrix test. Rank(C) = dim(A)
 Ctr = [ B A*B A^2*B A^3*B];
 rank(Ctr);
 
 % Eigen vector test. rank([A-lambdaI, B]) = dim(A) for every eigen value
 [V,D] = eig(A);
 for i = 1:4
    rank([A-D(i,i)*eye(4), B]);
 end
 
 % b) Can the system still be controlled if the radial thruster fails? What
 % if the tangential thruster fails?
 
 % radial thruster test
 br = B(:,1);
 Ctr_r = [ br A*br A^2*br A^3*br];
 rank(Ctr_r)
 
 % tangential thruster test
 bt = B(:,2);
 Ctr_t = [ bt A*bt A^2*bt A^3*bt];
 rank(Ctr_t)
 
 %% Problem 2
 
 % a) Linearize the system around the equilibrium point x1 = x2 =x3 =0. Is
 % the system controllable.
 
 A = [-1 0 0; 0 -1 0; 0 0 0]; 
 B = [1 0; 0 1; 0 0];
 rank(ctrb(A,B))
 
 % b) Linearize the system around the equilbrium point x1=x2=x3 = 1. Is
 % this system controllable.
 A = [-1 0 0; 0 -1 0; -1 1 0]; 
 B = [1 0; 0 1; 1 -1];
 rank(ctrb(A,B))
 
 %% Problem 3
 
 % Consider an LTI system
 A = [-1 0; 0 -1];
 B = [-1;1];
 C = [1 0; 0 1];
 D = [2;1];
 
 % a) Is this system realizable
 rank(ctrb(A,B));
 
 % b ) Perform a similarity transformation to obtain a controllable
 % realization.
 
 Ctr = ctrb(A,B); % Get the first n_bar l.i. columns of the controllability matrix.
                  % Also, get the null space of Ctr.
                 
 T = [Ctr(:,1),[1;1]];  % Create the similarity transformation
 
 A_bar = inv(T)*A*T;
 B_bar = inv(T)*B;
 C_bar = C*T;
 
 % Verify
 
 syms s
 C*inv(s*eye(2) -A)*B;
 
 C_bar(:,1)*inv(s - A_bar(1,1))*B_bar(1)
 
 C_bar*inv(s*eye(2)-A_bar)*B_bar