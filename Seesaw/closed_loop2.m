    main;

% output y equals measured quantities [x; theta]
I4 = eye(4);
C = I4(1:2,:);
D = zeros(2,1);


R = 1; % We change Q relative to R

% State vector x = [x  theta  dx  dtheta]'

% Default
Q = diag([100 400 0 0]);

% used for practicum:
%Q0 = diag([400 2000 0 0]);
%Q1 = diag([800 5000 0 0]);



K = lqr(A,B,Q,R);
clp = eig(A-B*K); % Closed loop poles


% Measurement noise, in m and rad respectively
sigma_x = 7.5e-5;
sigma_theta = 7.5e-5;

% Quantizer: 20V range with 16 bits
Vq = 20/2^16; % 1 quantization step
m_per_V = 0.456/4.41;
deg_per_V = 15/3.166;
rad_per_V = pi/180*deg_per_V;
% Quantization steps for position and angle
xq = Vq*m_per_V;
thetaq = Vq*rad_per_V;

Ts = 0.005;
f_c = 2 ;
omega_c = 2*pi*f_c;
alfa = omega_c*Ts/(1+omega_c*Ts);

f_c2 = 10;
omega_c2 = 2*pi*f_c2;
beta = omega_c2*Ts/(1+omega_c2*Ts);

