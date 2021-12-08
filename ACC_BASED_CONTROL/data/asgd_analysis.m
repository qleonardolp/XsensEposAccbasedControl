close all
clc
figure,
% data = asgd001;
plot(data.time, data.roll)
hold on
plot(data.time, data.pitch)
plot(data.time, data.yaw)
grid on

figure,
data = asgd002;
plot(data.time, data.roll)
hold on
plot(data.time, data.pitch)
plot(data.time, data.yaw)
grid on

figure,
data = asgd003;
plot(data.time, data.roll)
hold on
plot(data.time, data.pitch)
plot(data.time, data.yaw)
grid on
%%
figure,
plot((MT003.sample - MT003.sample(500))*0.00833, MT003.pitch + 90)
hold on
data = asgd003;
plot(data.time, data.roll)
grid on
