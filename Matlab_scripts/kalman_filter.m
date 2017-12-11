close all;
clear all;

load('accelerometer_data.mat')

% figure(1);
% hold on; grid on;
% plot(time,acc_x_g,time,acc_y_g,time,acc_z_g)
% hold off;
% 
% figure(2);
% hold on; grid on;
% plot(time,gyro_x_dps,time,gyro_y_dps,time,gyro_z_dps);
% hold off;
% 
% offset = [3.48; -0.162; 0.177];
% gain = [1.17; 1.133; 1.14];
% mag_x_gauss = (mag_x_gauss - offset(1)) * gain(1);
% mag_y_gauss = (mag_y_gauss - offset(2)) * gain(2);
% mag_z_gauss = (mag_z_gauss - offset(3)) * gain(3);
% 
% figure(3);
% hold on; grid on;
% plot(time,mag_x_gauss,time,mag_y_gauss,time,mag_z_gauss)
% hold off;

% %% init
q = 0.009; r = 0.16;
Q = [1 0;0 1]*q;   % niepewnoœæ procesu (szumy procesowe i niepewnoœæ modelowania)
R = r;             % niepewnoœæ pomiaru (im mniejsze tym bardziej ufamy pomiarom)

u  = gyro_y_dps(10958:21000);           % sterowanie, czyli prêdkoœæ odczytana z ¿yroskopu
y  = atan2(acc_z_g(10958:21000),acc_x_g(10958:21000)) - pi/2;          % wyjœcie, czyli po³o¿enie odczytane z akcelerometru
dt = 0.001;
A = [1 -dt;0 1];
B = [dt; 0];
C = [1 0];

Pk_corr = Q; 

n = length(u); % iloœæ iteracji algorytmu
time_x = time(10958:21000);

%% initial conditions
xk_pred      = zeros(1,2); % stan przewidywany 
xk_corr      = zeros(n,2); % stan po korekcji

xk_corr_0 = [y(1);median(gyro_y_dps)];

%% algorithm

xk_corr_prev = xk_corr_0;

for k = 1 : n
    xk_pred = A*xk_corr_prev +B*u(k);
    Pk_pred = A*Pk_corr*transpose(A) + Q;
    
    error = y(k) - C*xk_pred;
    K = (Pk_pred*transpose(C))/(C*Pk_pred*transpose(C)+R);
    xk_corr(k,:) = xk_pred + K*error;
    
    Pk_corr = (eye(2) - K*C)*Pk_pred;
    xk_corr_prev = xk_corr(k,:)';
end

x_estimate = xk_corr;

%% plot results
figure(4); hold on; grid on;
plot(time_x,y * 180 / pi)
plot(time_x,x_estimate(:,1) * 180 / pi,'r-');
xlabel('Czas [s]');ylabel('Po³o¿enie k¹towe [deg]');
legend('Po³o¿enie z akcelerometru','Estymata po³o¿enia');
hold off;

figure(5); hold on; grid on;
plot(time_x, u);
plot(time_x,x_estimate(:,2))
xlabel('Czas [s]');ylabel('Dryft ¿yroskopu [deg/s]');
legend('Dryft z ¿yroskopu','Estymata dryftu');
hold off;


u  = gyro_x_dps(5514:end);           % sterowanie, czyli prêdkoœæ odczytana z ¿yroskopu
y  = atan2(acc_z_g(5514:end),acc_y_g(5514:end)) - pi/2;          % wyjœcie, czyli po³o¿enie odczytane z akcelerometru
dt = 0.001;
A = [1 -dt;0 1];
B = [dt; 0];
C = [1 0];

Pk_corr = Q; 

n = length(u); % iloœæ iteracji algorytmu
time_y = time(5514:end);

%% initial conditions
xk_pred      = zeros(1,2); % stan przewidywany 
xk_corr      = zeros(n,2); % stan po korekcji

xk_corr_0 = [y(1);median(gyro_x_dps)];

%% algorithm

xk_corr_prev = xk_corr_0;

for k = 1 : n
    xk_pred = A*xk_corr_prev +B*u(k);
    Pk_pred = A*Pk_corr*transpose(A) + Q;
    
    error = y(k) - C*xk_pred;
    K = (Pk_pred*transpose(C))/(C*Pk_pred*transpose(C)+R);
    xk_corr(k,:) = xk_pred + K*error;
    
    Pk_corr = (eye(2) - K*C)*Pk_pred;
    xk_corr_prev = xk_corr(k,:)';
end

x_estimate = xk_corr;

%% plot results
figure(6); hold on; grid on;
plot(time_y,y * 180 / pi)
plot(time_y,x_estimate(:,1) * 180 / pi,'r-');
xlabel('Czas [s]');ylabel('Po³o¿enie k¹towe [deg]');
legend('Po³o¿enie z akcelerometru','Estymata po³o¿enia');
hold off;

figure(7); hold on; grid on;
plot(time_y, u);
plot(time_y,x_estimate(:,2))
xlabel('Czas [s]');ylabel('Dryft ¿yroskopu [deg/s]');
legend('Dryft z ¿yroskopu','Estymata dryftu');
hold off;


