clc
clear all
close all

current = importdata('2019-11-08-11-43-48_C.txt');
currentKF = importdata('2019-11-08-12-04-27_K.txt');
speed = importdata('2019-11-08-16-05-15_S.txt');
speed_exo = importdata('2019-11-08-19-13-40_S_exo.txt');
positionff = importdata('2019-11-08-19-26-45_P.txt');

dt = 0.008;     % time step

%% Plots of the Current Mode [C]

t = linspace(0, dt*length(current.data(:,1)), length(current.data(:,1)));

sg2_accHum = zeros(1000,1);

for i = 5501 : 6500
    sg2_accHum(i - 5500) = ( 5*current.data(i-1,8) +4*current.data(i-2,8) + 3*current.data(i-3,8)...
    +2*current.data(i-4,8) +1*current.data(i-5,8) +0*current.data(i-6,8) -1*current.data(i-7,8)...
    -2*current.data(i-8,8) -3*current.data(i-9,8) -4*current.data(i-10,8) -5*current.data(i-11,8)...
    )/110;
end

figure('Name', 'Current Setpoint in Current Mode')
plot(t(5500:6500), current.data([5500:6500],1),'-', 'LineWidth', 0.7)
hold on
plot(t(5500:6500), current.data([5500:6500],2),':', 'LineWidth', 1.2)
grid on
legend( current.colheaders{1,1} , char(current.textdata(1,2)) )
xlabel('time [s]')
hold off
%%
figure('Name', 'Human angular Vel and Acc [C]')
plot(t(5500:6500), current.data([5500:6500],6),'-', 'LineWidth', 0.7)
hold on
plot(t(5500:6500), current.data([5500:6500],8),':', 'LineWidth', 1.2)
hold on
plot(t(5501:6500), sg2_accHum(:,1))
grid on
legend( current.colheaders{1,6} , char(current.textdata(1,8)), 'acc_{hum} sg2')
xlabel('time [s]')
hold off

figure('Name', 'Speed measured with MTw on the Exo vs Motor Speed [C]')
plot(t(5500:6500), current.data([5500:6500],9),'-', 'LineWidth', 0.7)
hold on
plot(t(5500:6500), (1/150)*current.data([5500:6500],10),':', 'LineWidth', 1.2)
grid on
legend( char(current.textdata(9)) , char(current.textdata(10)) )
xlabel('time [s]')
hold off

figure('Name', 'Torques [C]')
plot(t(5500:6500), 104*(current.data([5500:6500],4) - current.data([5500:6500],3)),':r', 'LineWidth', 0.8)
hold on
plot(t(5500:6500), current.data([5500:6500],5), ':b')
grid on
legend('Torque SEA [N.m]', current.colheaders{1,5})
xlabel('time [s]')
hold off

%% Plots of Current Mode with Kalman Filter

t = linspace(0, dt*length(currentKF.data(:,1)), length(currentKF.data(:,1)));

figure('Name', 'Current Setpoint in CurrentKF Mode')
plot(t, currentKF.data(:,1),'-', 'LineWidth', 0.7)
hold on
plot(t, currentKF.data(:,2),':', 'LineWidth', 1.2)
grid on
legend( currentKF.colheaders{1,1} , currentKF.colheaders{1,2} )
xlabel('time [s]')
hold off

figure('Name', 'Human angular Vel and Acc [K]')
plot(t, currentKF.data(:,6),'-', 'LineWidth', 0.7)
hold on
plot(t, currentKF.data(:,8),':', 'LineWidth', 1.2)
grid on
legend( currentKF.colheaders{1,6} , currentKF.colheaders{1,8} )
xlabel('time [s]')
hold off

%% Plots for Speed Mode (using only one MTw)

t = linspace(0, dt*length(speed.data(:,1)), length(speed.data(:,1)));

figure('Name', 'Velocity Setpoint on Speed Mode [S]')
plot(t, speed.data(:,5))
hold on
plot(t, speed.data(:,6))
grid on
legend(char(speed.colheaders(5)), char(speed.colheaders(6)))
xlabel('time [s]')
hold off

figure('Name', 'Hum angular Vel and Acc [S]')
plot(t, speed.data(:,1))
hold on
plot(t, speed.data(:,3))
grid on
legend(char(speed.colheaders(1)), char(speed.colheaders(3)))
xlabel('time [s]')
hold off

figure('Name', 'Vel Hum X Vel Motor [S]')
plot(t, 200*speed.data(:,3))
hold on
plot(t, (1/150)*speed.data(:,5))
grid on
legend('k_{ff} vel_{hum} [rpm]', '(1/N)*vel_{motor} [rpm]')
xlabel('time [s]')
hold off

%% Plots for Speed Mode (using two MTw, Hum and Exo)

t = linspace(0, dt*length(speed_exo.data(:,1)), length(speed_exo.data(:,1)));

figure('Name', 'Velocity Setpoint on Speed Mode [S*]')
plot(t, speed_exo.data(:,5))
hold on
plot(t, speed_exo.data(:,6))
grid on
legend(char(speed_exo.colheaders(5)), char(speed_exo.colheaders(6)))
xlabel('time [s]')
hold off

figure('Name', 'Hum angular Vel and Acc [S*]')
plot(t, speed_exo.data(:,1))
hold on
plot(t, speed_exo.data(:,3))
grid on
legend(char(speed_exo.colheaders(1)), char(speed_exo.colheaders(3)))
xlabel('time [s]')
hold off

figure('Name', 'Vel Hum X Vel Motor [S*]')
plot(t, 200*speed_exo.data(:,3))
hold on
plot(t, (1/150)*speed_exo.data(:,5))
grid on
legend('k_{ff} vel_{hum} [rpm]', '(1/N)*vel_{motor} [rpm]')
xlabel('time [s]')
hold off

%% Plots of Position Mode [P]

t = linspace(0, dt*length(positionff.data(:,1)), length(positionff.data(:,1)));

figure('Name', 'Human angular Vel and Acc [P]')
plot(t, positionff.data(:,1))
hold on
plot(t, positionff.data(:,4))
grid on
legend( positionff.colheaders{1,1}, positionff.colheaders{1,3})
xlabel('time [s]')
hold off

figure('Name', 'Torque SEA [P]')
plot(t, 104*(positionff.data(:,6) - positionff.data(:,7)) )
grid on
legend('T_{sea} [N.m]')
xlabel('time [s]')
hold off

