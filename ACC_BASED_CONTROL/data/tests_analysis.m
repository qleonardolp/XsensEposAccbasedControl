clc
clear all
close all

dt = 0.008;     % time step
%% Current Mode [C]
t = linspace(0, dt*length(current.data(:,1)), length(current.data(:,1)));

%{
% Compute the FFT of the Hum Vel curve:
vH_fft = fft(current.data(5501:6500,8));
% Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 
% and the even-valued signal length L.
L = length(current.data(5501:6500,8));
P2 = abs(vH_fft/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
% Define the frequency domain f
f = (1/dt)*(0:(L/2))/L;

vH_Axf = [P1 f'];
[A_m, I] = max(vH_Axf,[],1);
A_vh = A_m(1);
f_vh = vH_Axf(I(1),2);
%}

accH_IL = zeros(1000,1); % using a Proportional Gain and in loop Integrator (IL)
f_cf = 1.6;
acc_IL_fb = 0;
acc_IL_last = zeros(4,1);

for i=5501 : 6500
    accH_IL(i-5500) = f_cf*(current.data(i,8) - acc_IL_fb);
    % Simpson Rule, h = (4*dt)/n, if n=4, h = dt
    acc_IL_fb = dt/3*(acc_IL_last(1) + 4*acc_IL_last(2) + 2*acc_IL_last(3) + 4*acc_IL_last(4) + accH_IL(i-5500)); % trapeziodal integral
    acc_IL_last(1) = acc_IL_last(2);
    acc_IL_last(2) = acc_IL_last(3);
    acc_IL_last(3) = acc_IL_last(4);
    acc_IL_last(4) = accH_IL(i-5500);
end

figure('Name', 'Current Mode 2020-01-25-16-25-19')
plot(t(5501:6500), current.data([5501:6500],1),'-', 'LineWidth', 0.7)
hold on
plot(t(5501:6500), current.data([5501:6500],2),':', 'LineWidth', 1.2)
grid on
legend( current.colheaders{1,1} , char(current.textdata(1,2)) )
title('Current Setpoint')
xlabel('time [s]')
hold off

figure('Name', 'Current Mode 2020-01-25-16-25-19')
plot(t(5501:6500), current.data(5501:6500,6),'-', 'LineWidth', 0.7)
hold on
plot(t(5501:6500), current.data(5501:6500,8),':', 'LineWidth', 1.2)
hold on
% from the FFT on the Hum Vel:
% plot(t(5501:6500), A_vh*cos( 2*pi*f_vh*t(5501:6500) )) % Hum Vel from the FFT
% plot(t(5501:6500), -2*pi*f_vh*A_vh*sin( 2*pi*f_vh*t(5501:6500) )) % Hum Acc from the vH_fft
plot(t(5501:6500), accH_IL(:,1))
grid on
legend( '*acc_h' , char(current.textdata(1,8)), 'acc_h IL')
title('Human angular Vel and Acc')
xlabel('time [s]')
hold off

figure('Name', 'Curent Mode 2020-01-25-16-25-19')
plot(t(5500:6500), current.data([5500:6500],9),'-', 'LineWidth', 0.7)
hold on
plot(t(5500:6500), (1/150)*current.data([5500:6500],10),':', 'LineWidth', 1.2)
grid on
legend( char(current.textdata(9)) , char(current.textdata(10)) )
title('Speed measured with MTw on the Exo vs Motor Speed')
xlabel('time [s]')
hold off

figure('Name', 'Current Mode 2020-01-25-16-25-19')
plot(t(5500:6500), 104*(current.data([5500:6500],4) - current.data([5500:6500],3)),':r', 'LineWidth', 0.8)
hold on
plot(t(5500:6500), current.data([5500:6500],5), ':b')
grid on
legend('Torque SEA [N.m]', current.colheaders{1,5})
title('Torques')
xlabel('time [s]')
hold off

%%  accBasedController
% abc_data = importdata('2021-02-28-19-10-01.txt');
% abc_data = importdata('abc_com_ID/2021-03-27-21-56-05.txt');
% abc_data = importdata('abc_acc_motor_/2021-03-28-09-51-40.txt');
% akf_data = importdata('cac_com_IDcorr_com_ff_fb/akf-2021-03-28-10-41-27.txt');
% abc_data = importdata('cac_com_IDcorr_com_ff_fb/2021-03-28-10-41-27.txt');
abc_data = importdata('abc_sem_ID/2021-03-27-21-59-14.txt');
% abc_data = importdata('cac_sem_ID_vel_hum/2021-03-27-23-05-07.txt');
abc_data = importdata('2021-05-08-19-36-25.txt');
akf_data = importdata('akf-2021-05-08-19-36-25.txt');
t_end = abc_data.data(end,1)
% figure, plot(abc_data.data(:,1), rad2deg([abc_data.data(:,2) abc_data.data(:,4)])), grid on
% legend('velHum','accHum')

% abc_sem_ID_log % Avg abs pos error = 1.745 deg (soft) (t 8.2 a 13.2)
% abc_sem_ID_log % Avg abs pos error = 1.292 deg (rapid) (t 34 a 49)

% cac_sem_ID_log % Avg abs pos error = 1.698 deg (soft) (t 11 a 16)
% cac_sem_ID_log % Avg abs pos error = 3.168 deg (rapid) (t 45 a 50)

%%
% close all

Kp = 0.5436;
Ki = 11.560;
figure, 
subplot(4,1,1)
plot(abc_data.data(:,1), rad2deg([abc_data.data(:,2) abc_data.data(:,3)])), grid on
legend('velHum','velExo'), ylabel('deg/s')

subplot(4,1,2)
plot(abc_data.data(:,1), Ks*(abc_data.data(:,6) - abc_data.data(:,7))), hold on
plot(abc_data.data(:,1), -We*Le*sin(abc_data.data(:,7))), grid on
legend('Tsea','T_W'), ylabel('N.m')

subplot(4,1,3)
plot(abc_data.data(:,1), rad2deg(abc_data.data(:,7))), grid on
legend('theta_{exo}'), ylabel('deg')

subplot(4,1,4)
des_current = -0*We*Le*sin(abc_data.data(:,7)) + Je*abc_data.data(:,4) + ...
                Kp*(abc_data.data(:,4) - abc_data.data(:,5)) + ...
                Ki*(abc_data.data(:,2) - abc_data.data(:,3));
            % + Ja*abc_data.data(:,9)
des_current = des_current/(N*KI);
CURRENT_MAX = 3.1400;
plot(abc_data.data(:,1), des_current), hold on
yline(CURRENT_MAX,'--r',{'I_{max}'})
yline(-CURRENT_MAX,'--r'), grid on
legend('Motor I_d'), ylabel('A')

%% AKF comparative
% obs: vel_act and m_current is probably missing on zk

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,2))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,7)))
title('Hum Vel')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,3))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,5)))
title('Exo Vel')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,8))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,6)))
title('Act Vel')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,4))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,8)))
title('Hum Acc')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,5))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,9)))
title('Exo Acc')

figure, plot(akf_data(:,1), akf_data(:,10)), grid on, title('Int Torque')