
load references_14.mat

m=0.5; L=0.25; k=3e-6; b=1e-7; g=9.81; kd=0.25;
Ixx=5e-3; Iyy=5e-3; Izz=1e-2; cm=1e4;

% NOTE in steady state: sum(v_i^2)*k*cm/m = g
% Moreover, v_i^2 = sum(v_i^2)/4 for all rotors
sum_vi2 = g*m/k/cm;
vi2 = sum_vi2/4;

u_ss = [vi2; vi2; vi2; vi2];
x_ss = zeros(12,1);
y_ss = zeros(6,1);

A = [
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 -kd/m 0 0 0  g 0 0 0 0;
    0 0 0 0 -kd/m 0 -g 0 0 0 0 0;
    0 0 0 0 0 -kd/m 0  0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 
];

B = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    k*cm/m k*cm/m k*cm/m k*cm/m;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    L*k*cm/Ixx  0           -L*k*cm/Ixx  0;
    0           L*k*cm/Iyy  0            -L*k*cm/Iyy;
    b*cm/Izz    -b*cm/Izz   b*cm/Izz     -b*cm/Izz;
];

I12 = eye(12);
C = I12([1:3 7:9],:);
D = zeros(6,4);

sys = ss(A,B,C,D);

Ts = 0.05;
G_d = c2d(sys,Ts,'zoh');
Az = G_d.A; Bz = G_d.B; Cz = G_d.C; Dz = G_d.D;

% Full state feedback
N = [Az-eye(12), Bz; Cz Dz]\[zeros(12,6); eye(6)];
Nx = N(1:12,:);
Nu = N(13:end,:);
