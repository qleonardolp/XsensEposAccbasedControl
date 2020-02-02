clc
clear all
close all

onlyEPOS_minus_vel_exo = importdata('2020-02-02-15-48-37_minus_vel_exo.txt');
onlyEPOS_vel_exo = importdata('2020-02-02-15-52-45_vel_exo.txt');
seaTfromEPOS = importdata('2020-02-02-19-14-41.txt');
seaTfromMTwEPOS = importdata('2020-02-01-22-02-29_CAC_seaTfromMTwEPOS.txt');


dt = 0.008;     % time step

%% Collocated Admittance Control [CAC]

t = linspace(0, dt*length(onlyEPOS_minus_vel_exo.data(:,1)), length(onlyEPOS_minus_vel_exo.data(:,1)));

figure('Name', 'CAC')
plot(t, onlyEPOS_minus_vel_exo.data(:,1),'-', 'LineWidth', 0.7)
hold on
plot(t, onlyEPOS_minus_vel_exo.data(:,2),'-', 'LineWidth', 0.7)
hold on
plot(t, 1/150*onlyEPOS_minus_vel_exo.data(:,4),'-', 'LineWidth', 0.7)
grid on
legend( onlyEPOS_minus_vel_exo.colheaders{1,1} , char(onlyEPOS_minus_vel_exo.textdata(1,2)), char(onlyEPOS_minus_vel_exo.textdata(1,4)) )
title('CAC Velocities (velexo = - m_eixo_out)')
xlabel('time [s]'), ylabel('rad/s')
hold off

t = linspace(0, dt*length(onlyEPOS_vel_exo.data(:,1)), length(onlyEPOS_vel_exo.data(:,1)));

figure('Name', 'CAC')
plot(t, onlyEPOS_vel_exo.data(:,1),'-', 'LineWidth', 0.7)
hold on
plot(t, onlyEPOS_vel_exo.data(:,2),'-', 'LineWidth', 0.7)
grid on
legend( onlyEPOS_vel_exo.colheaders{1,1} , char(onlyEPOS_vel_exo.textdata(1,2)) )
title('CAC Velocities (velexo = m_eixo_out)')
xlabel('time [s]'), ylabel('rad/s')
hold off

%%  T_Sea analysis

t = linspace(0, dt*length(seaTfromEPOS.data(:,1)), length(seaTfromEPOS.data(:,1)));

figure('Name', 'CAC')
plot(t, seaTfromEPOS.data(:,3),'-', 'LineWidth', 0.7)
grid on
legend('SeaT EPOS')
xlabel('time [s]'), ylabel('N.m')
hold off

%% Derivative analysis

OmegaControl = importdata('2020-02-02-17-32-56.txt');

t = linspace(0, dt*length(OmegaControl.data(:,1)), length(OmegaControl.data(:,1)));

figure('Name', 'OmegaControl using Jerk (Cutoff = 30)')
plot(t, OmegaControl.data(:,3),'-', 'LineWidth', 0.7)
hold on
plot(t, OmegaControl.data(:,1),'-', 'LineWidth', 0.7)
hold on
plot(t, OmegaControl.data(:,5),'-', 'LineWidth', 0.7)
grid on
legend( 'vel hum' , 'acc hum', 'jerk hum' )
xlabel('time [s]')
hold off


figure('Name', 'OmegaControl using Jerk')
plot(t, OmegaControl.data(:,10),'-', 'LineWidth', 0.7)
hold on
plot(t, OmegaControl.data(:,11),'-', 'LineWidth', 0.7)
hold on
plot(t, OmegaControl.data(:,12),'-', 'LineWidth', 0.7)
grid on
legend( ' T Sea' , 'theta c', 'theta l' )
xlabel('time [s]')
hold off

