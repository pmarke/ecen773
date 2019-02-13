%% Aircraft Dynamics
% x_dot = A*x + B*u

P.A = [-0.038   18.984   0      -32.174;...
     -0.001  -0.632    1       0;...
     0       -0.759   -0.518   0;...
     0        0        1       0];

P.B =   [10.1       0;...
      0        -0.0086;...
      0.025    -0.011;...
      0         0];
  
%% a. Compute the modes of this system
% The mode is defined as exp(lambda_i*t)Vi
[V,lambda] = eig(P.A);

%% b. Analyze these modes physically by looking at how the physical states play a role in each mode. 
% Are there some modes that deal more with the pitch? Others that have more
% effect on the angle of attack?
V_r = [0.9953    0.9953    1.0000    1.0000;...
       0.0562    0.0562   -0.0005   -0.0005;...
      -0.0191   -0.0191    0.0007    0.0007;...
       0.0500    0.0500   -0.0011   -0.0011];

lambda_r = [-0.5825 ;...
            -0.5825 ;...
            -0.0115 ;...
            -0.0115];...

%  We can analyze these modes physically by looking at their real parts. If 
%  the initial state of the aircraft is along the span of any of thses
%  modes, then the states of the aircraft will evolve as X(t) = sigma
%  exp(lambda_i*t)X(0) where X(0) = sigma*V_i and sigma is a scalar. By
%  looking at the eigen vector associated with the mode, we can see how
%  some physical states are affected vs other physical states. In all of
%  the modes, velocity is affected the most. The first two modes seem to have 
%  more affect on the pitch and and angle of attack than the other two
%  modes.

%% C. Aircraft generally exhibit two longitudinal motions (also called modes),
% a phugoid and short period mode. Phugoid represents the coupling between
% the vehicle altitude and the airspeed, while the short period mode (whith
% much faster dynamics) is the coupling between the angle of attack and the
% pitch rate. Identify the values from (a) that best represent the phugoid
% and short period modes respectively. 

%  The first two modes show a strong coupling between the angle of attack
%  and the pitch rate, these must be the short period modes. By default the
%  other two modes must be the phugoid modes. ?????????????????

%% D. Given an input this aircraft wil respond along a linear combination 
% of the modes. Determine how strongly each of the inputs will effect each
% of the different modes. Will one input effect one physical parameter more
% strongly? 

        
% All of the real parts of the eigen values of A are negative, thus
% the system is exponentially stable.



      
 