main;

% output y equals state vector x
C = eye(4);
D = zeros(4,1);

R = 1; % We change Q relative to R

% State vector x = [x  theta  dx  dtheta]'
% Q = diag([100 400 0 0]);
Q = diag([50 300 50 50]);

% used for practicum:
%Q = diag([400 2400 0 0]);




K = lqr(A,B,Q,R);

clp = eig(A-B*K); % Closed loop poles
