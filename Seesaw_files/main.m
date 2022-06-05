Rm = 2.6;
Kt = 0.00767;
Km = 0.00767;
Kg = 3.71;
eta_m = 1;
eta_g = 1;
Mc = 0.52;
Msw = 3.6;
% Jsw is the moment of inertia about the center of gravity, should be about
% pivot point to match theory! I guess there isn't a big difference?
Jsw = 0.3950;
Dt = 0.1250;
Dc = 0.058;
r = 0.00635;
g = 9.81;
Beq =  0.9;
Bsw = 0;

a31 = -Mc*Dt*g/Jsw;
a32 = (-g*Mc*Rm*r^2*Jsw + Mc*Dt*Rm*r^2*g*Msw*Dc)/(Rm*r^2*Jsw*Mc);
a33 = (-Jsw*eta_g*Kg^2*eta_m*Kt*Km - Jsw*Beq*Rm*r^2 - Mc*Dt^2*eta_g*Kg^2*eta_m*Kt*Km - Mc*Dt^2*Beq*Rm*r^2)/(Rm*r^2*Jsw*Mc);
a34 = -Dt*Bsw/Jsw;
a41 = -g*Mc/Jsw;
a42 = g*Msw*Dc/Jsw;
a43 = (-eta_g*Kg^2*eta_m*Kt*Km*Dt - Beq*Rm*r^2*Dt)/(Rm*r^2*Jsw);
a44 = -Bsw/Jsw;
b3 = (Jsw*eta_g*Kg*eta_m*Kt*r + Mc*Dt^2*eta_g*Kg*eta_m*Kt*r)/(Rm*r^2*Jsw*Mc);
b4 = eta_g*Kg*eta_m*Kt*Dt/(r*Rm*Jsw);

A = [0   0   1   0;
     0   0   0   1;
     a31 a32 a33 a34;
     a41 a42 a43 a44];
B = [0; 0; b3; b4];
