main;

% output y equals [x; theta]
I4 = eye(4);
C = I4(1:2,:);
D = zeros(2,1);

sys = ss(A,B,C,D);

[P, Z] = pzmap(sys);
% System is unstable

CO = ctrb(A,B);
rank_OC = rank(CO);
% System is controllable

Ob = obsv(A,C);
rank_Ob = rank(Ob);
% System is observable

% System is also stabilizable, since unstable mode is controllable
% System is also detectable, since unstable mode is observable
% System is also minimal, since it's controllable and observable
