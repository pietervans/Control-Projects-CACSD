main;

% output y equals state vector x
C = eye(4);
D = zeros(4,1);

Q = diag([100 400 0 0]);
R = 1;
K = lqr(A,B,Q,R);


% USED IN SIMULINK DIAGRAM
% function x_desired = get_x_desired(theta_desired)
%     % From equation (13)
%     x_desired = (Mc*Dt+Msw*Dc)*tan(theta_desired)/Mc;
% end
