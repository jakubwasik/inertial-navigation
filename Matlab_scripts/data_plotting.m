close all;
clear all;

%% Small range of angles
load('workspaces\orientation_estimation_low_angles.mat')

time = orientation_estimation.time;

pitch = orientation_estimation.signals(1).values(:,1);
pitch_estimate = orientation_estimation.signals(1).values(:,2);

roll = orientation_estimation.signals(2).values(:,1);
roll_estimate = orientation_estimation.signals(2).values(:,2);

yaw = orientation_estimation.signals(3).values(:,1);
yaw_estimate = orientation_estimation.signals(3).values(:,2);

figure(1);
hold on; grid on;
plot(time,pitch,time,pitch_estimate);
title('Pitch - ma³y zakres k¹tów');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(2);
hold on; grid on;
plot(time,roll,time,roll_estimate);
title('Roll - ma³y zakres k¹tów');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(3);
hold on; grid on;
plot(time,yaw,time,yaw_estimate);
title('Yaw - ma³y zakres k¹tó');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

%% Large range of angles with a low frequency of changes
load('workspaces\orientation_estimation_high_angles_low_freq.mat')

time = orientation_estimation.time;

pitch = orientation_estimation.signals(1).values(:,1);
pitch_estimate = orientation_estimation.signals(1).values(:,2);

roll = orientation_estimation.signals(2).values(:,1);
roll_estimate = orientation_estimation.signals(2).values(:,2);

yaw = orientation_estimation.signals(3).values(:,1);
yaw_estimate = orientation_estimation.signals(3).values(:,2);

figure(4);
hold on; grid on;
plot(time,pitch,time,pitch_estimate);
title('Pitch - du¿y zakres k¹tów z ma³¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(5);
hold on; grid on;
plot(time,roll,time,roll_estimate);
title('Roll - du¿y zakres k¹tów z ma³¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(6);
hold on; grid on;
plot(time,yaw,time,yaw_estimate);
title('Yaw - du¿y zakres k¹tów z ma³¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;


%% Large range of angles with a high frequency of changes

load('workspaces\orientation_estimation_high_angles_high_freq.mat')

time = orientation_estimation.time;

pitch = orientation_estimation.signals(1).values(:,1);
pitch_estimate = orientation_estimation.signals(1).values(:,2);

roll = orientation_estimation.signals(2).values(:,1);
roll_estimate = orientation_estimation.signals(2).values(:,2);

yaw = orientation_estimation.signals(3).values(:,1);
yaw_estimate = orientation_estimation.signals(3).values(:,2);

figure(7);
hold on; grid on;
plot(time,pitch,time,pitch_estimate);
title('Pitch - du¿y zakres k¹tów z du¿¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(8);
hold on; grid on;
plot(time,roll,time,roll_estimate);
title('Roll - du¿y zakres k¹tów z du¿¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(9);
hold on; grid on;
plot(time,yaw,time,yaw_estimate);
title('Yaw - du¿y zakres k¹tów z du¿¹ czêstotliwoœci¹ zmian');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;
