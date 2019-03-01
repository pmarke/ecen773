%% Problem 2

A = [0    1  0         0;...
     0 -0.1 -0.98      1;...
     0    0  0         1;...
     0  0.1  10.78   -11];
 
 B = [0     0;...
      0.1  -0.1;...
      0     0;...
     -0.1   1.1];
 
 %% a)
 % Compute the modes of the system
 [V,J] = eig(A);
 
 %% c)
 % Describe how much each input effects each of the modes
 w = inv(V);
 
 % Force affect on modes
 F_M1 = w(1,:)*B(:,1);       % First mode
 F_M2 = w(2,:)*B(:,1);       % Second mode
 F_M3 = w(3,:)*B(:,1);       % Third mode
 F_M4 = w(4,:)*B(:,1);       % Fourth mode
 
 % Torque affect on modes
 T_M1 = w(1,:)*B(:,2);      % First mode
 T_M2 = w(2,:)*B(:,2);       % Second mode
 T_M3 = w(3,:)*B(:,2);       % Third mode
 T_M4 = w(4,:)*B(:,2);       % Fourth mode
 
 %% d) 
 % Determine an initial condition xo such that if x(0) = xo, then x(t)
 % -> 0 ans t -> inf;
 % We need xo to not affect the nodes where the eigen values are non zero.
 % this can be done by constucting xo to be in the null space of wk that
 % corresponds to the unstable nodes.
 
 wn = [w(1,:);w(4,:)];    % The two vectors coresponding to the unstable nodes
 [~,~,v] = svd(wn);       % Compute the svd to get the null space
 vn = v(:,3:4);           % Extract the null space
 P = vn*vn'/norm(vn*vn'); % Create a projection matrix that will project any 
                          % vector onto the null space of wn. 
                          
 % Test
%  x_array = P*randn(4,10);
%  temp = w*x_array;
%  temp = round(temp,6,'significant');
% %  temp(4,:) = 0;
%  expm(J*80)*temp   %machine precision error
 
%% e
% If the rank of the controllability matrix is the same as the number of
% states, then the system is fully controllable.

% Controllability matrix with only force
CO_F = ctrb(A,B(:,1));
rank(CO_F);

% Controllability matrix with only torque
CO_T = ctrb(A,B(:,2));
rank(CO_T);