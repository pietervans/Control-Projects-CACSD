main;

% output y equals state vector x
C = eye(4);
D = zeros(4,1);

R = 1; % We change Q relative to R

% State vector x = [x  theta  dx  dtheta]'
% Q = diag([100 400 0 0]);
Q = diag([50 300 50 50]);

% Try to stay within these limits:
% |u| < 3.5V (can be increased to 5V)
% |x| < ?  --> what is length of track?


K = lqr(A,B,Q,R);

clp = eig(A-B*K); % Closed loop poles
