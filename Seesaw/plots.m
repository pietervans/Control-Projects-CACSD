
% Q0 = diag([400 2000 0 0]);
% Q1 = diag([800 5000 0 0]);

%% Step response 1° with Q0
[tt, uu, xx, theta] = load_data('data/p_Q0_0-1_square.mat');
rr = 8200:11900;
figure, plot(tt(rr), xx(rr,:))
figure, plot(tt(rr), theta(rr,:))

%% Step response 2° with Q0
[tt, uu, xx, theta] = load_data('data/p_Q0_0-2_square.mat');
rr = 4050:7950;
figure, plot(tt(rr), xx(rr,:))
figure, plot(tt(rr), theta(rr,:))

%% Step response 1° with Q1
[tt, uu, xx, theta] = load_data('data/p_Q1_0-1_square.mat');
rr = 1:3950;
figure, plot(tt(rr), xx(rr,:))
figure, plot(tt(rr), theta(rr,:))

%% Step response 2° with Q1
[tt, uu, xx, theta] = load_data('data/p_Q1_0-2_square.mat');
rr = 1:3950;
figure, plot(tt(rr), xx(rr,:))
figure, plot(tt(rr), theta(rr,:))

%% Disturbance with Q1
[tt, uu, xx, theta] = load_data('data/p_Q1_disturbance.mat');

%% f_cut-off = 8 Hz
[tt, uu, xx, theta] = load_data('data/p_Q1_fc8.mat');
rr = 1000:4000;
plot(tt(rr), uu(rr)) % Lots of noise in input signal

%% f_cut-off = 0.5 Hz
[tt, uu, xx, theta] = load_data('data/p_Q1_fc0p5.mat');
plot(tt, theta) % Too much filtering, large oscillations



function [tt, uu, xx, theta] = load_data(name)
data = load(name);
tt = data.data_ss.time;
signals = data.data_ss.signals;
uu = signals(1).values;
xx = signals(2).values;
theta = signals(3).values;
end