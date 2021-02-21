%% State Space formulation for Transparency Control with Rotary SEA.
% velHum as a control input
% Leonardo Felipe L. S. dos Santos, 2020, EESC@USP

% Constants
W = 4.742065; % Kg
g   = 9.80665;       % m/ss
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % Nm/rad
Ka  = Ks/20;          % Nm/rad (???)
Jh  = 0.05;          % Kg.m^2 (???)
Le  = 0.40;          % m      (???)
Je  = W*Le^2;        % Kg.m^2 
We  = W*g;           % N      
Beq = 60;            % N.m.s/rad (maybe is 30, I.O.)
Ja  = 0.47;          % Kg.m^2

% State vector is  [x_h x_e x_a \dot{x_e} \dot{x_a}]
% Sensor vector is [\dot{x_h} x_e x_m \dot{x_e} \dot{x_m}]
state_dim  = 5;
sensor_dim = 5;
control_dim = 3;

% State Space matrices

A = [zeros(1,state_dim); ...
    0 0 0 1 0; ...
    0 0 0 0 1; ...
    Ka/Je -(Ks + Ka)/Je Ks/Je 0 0; ...
    0 Ks/Ja -Ks/Ja 0 -Beq/Ja];

B = [1 0 0; zeros(2,control_dim); 0 -1/Je 0; 0 0 N*KI/Ja];

C = [eye(sensor_dim)]; C(1,1) = 0; C(3,3) = N; C(5,5) = N;
D = zeros(sensor_dim, control_dim); D(1,1) = 1;

poles = eig(A)  % check open-loop stability

% Check Controllability & Observability:
if (rank(ctrb(A,B)) == state_dim) && (rank(obsv(A,C)) == state_dim)
    disp('System is controllable and observable.')
elseif (rank(ctrb(A,B)) == state_dim) && (rank(obsv(A,C)) ~= state_dim)
    disp('System is controllable and not observable.')
elseif (rank(ctrb(A,B)) ~= state_dim) && (rank(obsv(A,C)) == state_dim)
    disp('System is not controllable and is observable.')
else
    disp('System is neither controllable and observable.')
end
%% State Space definition, 
% considering the different sensors 
% sample rate as output delay:
imu_delay = 8; % [ms]
Sys = ss(A,B,C,D,'TimeUnit','milliseconds','OutputDelay',[imu_delay 0 imu_delay 0 0]');
% Discretization
Ts = 1; % [ms]
% sea_bw_freq = 9.6/1000; % [kHz]
% disOpt = c2dOptions('Method','tustin','PrewarpFrequency',2*pi*sea_bw_freq);
% discreteSys = c2d(Sys,Ts,disOpt)
discreteSys = c2d(Sys,Ts)

%% Control Design, CAC
epos_Ki = 1.190; epos_Kp = 11.900;

damping_d = 0.900;
stiffness_d = 3.874;
stiffness_lower = 3.8422;

eposVelocityController = pid(epos_Kp,epos_Ki,0,0,0.001);

s = tf('s')

CAC = (1 - stiffness_d/Ks)*s / (stiffness_d + s*damping_d);
CAC = c2d(CAC,0.001);

stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*(1 - stiffness_d/Ks)) - epos_Kp/Ja)

%% Tuning from the characteristic equation (acceleration-based)

w_n = 2.165;      % Hz
T_s = 4/(1*w_n);  % s

w_max = sqrt(Ka/Je);
% From the Canonical form:
Kp_acc = (Ka - Je*w_n^2)/(w_n^2);
% Forcing critically damped response (zeta = 1):
Ki_acc = 2*sqrt((Kp_acc + Je)*Ka);

fprintf("Setting time: %.4f\nMax natural freq: %.4f\nKp: %.4f\nKi: %.4f\n",T_s,w_max,Kp_acc,Ki_acc);
