%% Question 1.
clear

%%% a)
% Find a realization of the transfer function.
% Verifying asnwers
A = [-2 0 0 0;...
      0 -2 0 0;...
      1 0 0 0;...
      0 1 0 0];
  
 B = [1 0;...
      0 1;...
      0 0;...
      0 0];
  
 C = [1 1 2 0];
 
 D = [1 0];
 
 syms s; 
 tf3 = C*inv(s*eye(4) - A)*B + D; 
 simplify(tf3);
 
 %%% b) Determine if this realization is controllable and/or observable
 ctrb_AB = rank(ctrb(A,B))
 obvs_AC = rank(obsv(A,C))
 
 %%% c) 
 A2 = [-2 0; 1 0];
 B2 = [1;0];
 C2 = [ 1 2; 1 0];
 D2 = [1;0]
 
simplify(C2*inv(s*eye(2)-A2)*B2 + D2);
ctrb_AB_new = rank(ctrb(A2,B2))
Obvs_AC_new = rank(obsv(A2,C2))

%% Question 3
clear
A = [1  0  0;...
     1  1  0;...
     -2 1 1];
B = [2 0 0]';
C = [1 0 1];

%%% a) Is this system observable?
rank(obsv(A,C));

%%% b) Compute a matrix k such that A+kc has the eigen values of -1
syms s k1 k2 k3 real;
K = [k1 k2 k3]';
temp = A+K*C;            % Put this in control cononical form
alpha = charpoly(temp);
Cr = [B temp*B, temp^2*B];
T = Cr*[1 alpha(2) alpha(3); 0 1 alpha(2); 0 0 1];
ACc = simplify(inv(T)*temp*T);

% We need the first row of ACc to equal [-3 -3 -1] IOT move the poles
a = [1 0 1; -4 1 -2; 4 -1 1];
b = [-6 0 -2]';
K = [a \ b];

%%% c)
f = [-9 -74 -24];

temp = A-B*k1*C;
alpha = charpoly(temp);
Cr = [B temp*B, temp^2*B];
T = Cr*[1 alpha(2) alpha(3); 0 1 alpha(2); 0 0 1];
ACc = simplify(inv(T)*temp*T);


%% Question 4
clear

A = [-1 0; 0 -1];
B = eye(2);
C = [-1 1];
D = [2 1];
% A realization is minimal if it is both observable and controllable
ctrb_AB = rank(ctrb(A,B))
obvs_AC = rank(obsv(A,C))
T = [-1 1; 1 1];
A_bar = inv(T)*A*T;
B_bar = inv(T)*B;
C_bar = C*T;
Aoc = A_bar(1,1);
Boc = B_bar(1,:);
Coc = C_bar(1);

syms s
simplify(C*inv(s*eye(2)-A)*B + D)
simplify(Coc*inv(s-Aoc)*Boc + D)

%% Question 5
clear
syms a1 a2 a3 a4 c1 c2 c3 c4 b1 b2 b3 b4 real
A = diag([a1,a2,a3,a4]);
C = [c1 c2 c3 c4];
B = [b1 b2 b3 b4]';

%%% d)

A = [1 1; 1 0];
C = [1 0];
B = [1;0];

rank(ctrb(A,B))
rank(obsv(A,C))

