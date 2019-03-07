function dxdt = aircraftDynamics(t,x,A,be,zeta)



u = be'*expm(A'*(60.00-t))*zeta;
dxdt = A*x + be*u;

end