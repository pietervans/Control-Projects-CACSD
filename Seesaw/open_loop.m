main;

% output y equals [x; theta]
I4 = eye(4);
C = I4(1:2,:);
D = zeros(2,1);

sys = ss(A,B,C,D);
pzmap(sys)