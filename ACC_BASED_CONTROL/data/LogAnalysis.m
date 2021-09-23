clc
clear all
close all

% abc_data = importdata('2021-02-28-19-10-01.txt');
% abc_data = importdata('abc_com_ID/2021-03-27-21-56-05.txt');
% abc_data = importdata('abc_acc_motor_/2021-03-28-09-51-40.txt');
% akf_data = importdata('cac_com_IDcorr_com_ff_fb/akf-2021-03-28-10-41-27.txt');
% abc_data = importdata('cac_com_IDcorr_com_ff_fb/2021-03-28-10-41-27.txt');
% abc_data = importdata('abc_sem_ID/2021-03-27-21-59-14.txt');
% abc_data = importdata('cac_sem_ID_vel_hum/2021-03-27-23-05-07.txt'); % !!
% akf_data = importdata('cac_sem_ID_vel_hum/akf-2021-03-27-23-05-07.txt'); %!!
% abc_data = importdata('2021-05-08-19-49-00.txt'); %usado para o texto do TCC
% akf_data = importdata('akf-2021-05-08-19-49-00.txt'); %usado para o texto do TCC
abc_data = importdata('2021-09-14-19-22-41.txt');
akf_data = importdata('akf-2021-09-14-19-22-41.txt');
t_end = abc_data.data(end,1) 
length(akf_data(:,1))/t_end*4

% figure, plot(abc_data.data(:,1), rad2deg([abc_data.data(:,2) abc_data.data(:,4)])), grid on
% legend('velHum','accHum')

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

%% KF comparative
% obs: vel_act and m_current is probably missing on zk

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,2))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,7)))
plot(akf_data(:,1), rad2deg(akf_data(:,8)))
title('Hum Vel')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,3))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,5)))
plot(akf_data(:,1), rad2deg(akf_data(:,9)))
title('Exo Vel')

figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,8))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,6)))
title('Act Vel')

% Usando no texto do Tcc:
%%
fd_acc = zeros(1, 2);
j = 1;
for i = 1:length(abc_data.data(:,4))
    if abc_data.data(i,4) ~= 0
        fd_acc(j,1) = abc_data.data(i,1);
        fd_acc(j,2) = abc_data.data(i,4);
        j = j+1;
    end
end

clc, close all
figure('Name','Angular acceleration','Color',[1 1 1]),
plot(fd_acc(:,1), rad2deg(fd_acc(:,2)),...
     'LineWidth',1.0), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,8)),'-.',...
     'LineWidth',2.0)
ax = gca;
ax.FontSize = 12; ax.LineWidth = 0.7; ax.GridAlpha = 0.5;
xlabel('time (s)'), ylabel('deg/s^2')
legend('FD','Kalman','Orientation','horizontal')
% title('Human Acceleration')


figure, plot(abc_data.data(:,1), rad2deg(abc_data.data(:,5))), grid on
hold on,
plot(akf_data(:,1), rad2deg(akf_data(:,9)))
xlabel('time (s)')
title('Exo Acceleration')

figure, plot(akf_data(:,1), akf_data(:,10)), grid on, title('Int Torque')