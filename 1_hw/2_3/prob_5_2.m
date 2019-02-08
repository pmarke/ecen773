syms x1_0 x2_0 t t0 tau

Phi = [1 1/4*(1-2*t0 -(1-2*t)*exp(2*(t-t0))); 0 exp(2*(t-t0))]
x0 = [x1_0;x2_0]
Phi2 = [1 1/4*(1-2*tau -(1-2*t)*exp(2*(t-tau))); 0 exp(2*(t-tau))]
B = [0; tau];
t0 = 0;
X = subs(Phi*x0) + int(Phi2*B,tau,0,t)
C = [1 0];

y = C*X;
simplify(y)