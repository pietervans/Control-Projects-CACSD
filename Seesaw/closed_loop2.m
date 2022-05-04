closed_loop1;

% output y equals measured quantities [x; theta]
I4 = eye(4);
C = I4(1:2,:);
D = zeros(2,1);

% TODO Idea: in closed_loop2_simulink: use filtered measurement for control actions?

% TODO  model measurement noise?
% Noise: which amplitude?

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

