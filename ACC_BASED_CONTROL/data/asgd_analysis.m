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
plot(data.time, data.roll2)
hold on
plot(data.time, data.pitch2)
plot(data.time, data.yaw2)
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

%%
figure,
plot((MT001.sample - MT001.sample(2510))*0.00833, MT001.pitch)
hold on
plot(asgd.time - 60, asgd.pitch2)
grid on

%%
figure,
plot(asgd(:,1), asgd(:,5))
hold on
plot((MT002(:,1) - MT002(1,1))*0.00833, MT002(:,3))
