close all;
clear all;

%% Filter ok
load('workspaces\orientation_estimation.mat')

time = orientation_estimation.time;

pitch = orientation_estimation.signals(1).values(:,1);
pitch_estimate = orientation_estimation.signals(1).values(:,2);

roll = orientation_estimation.signals(2).values(:,1);
roll_estimate = orientation_estimation.signals(2).values(:,2);

yaw = orientation_estimation.signals(3).values(:,1);
yaw_estimate = orientation_estimation.signals(3).values(:,2);

figure(1);
hold on; grid on;
plot(time(20000:33000) - 119.99,pitch(20000:33000),time(20000:33000) - 119.99,pitch_estimate(20000:33000));
title('Pitch');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

figure(2);
hold on; grid on;
plot(time(20000:39000) - 119.99,roll(20000:39000),time(20000:39000) - 119.99,roll_estimate(20000:39000));
title('Roll');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

figure(3);
hold on; grid on;
plot(time(5000:19000) - 49.99,yaw(5000:19000),time(5000:19000) - 49.99,yaw_estimate(5000:19000));
title('Yaw');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

%% To low filter frequency
load('workspaces\orientation_estimation_to_low_filter_freq.mat')

time = orientation_estimation.time;

pitch = orientation_estimation.signals(1).values(:,1);
pitch_estimate = orientation_estimation.signals(1).values(:,2);

roll = orientation_estimation.signals(2).values(:,1);
roll_estimate = orientation_estimation.signals(2).values(:,2);

yaw = orientation_estimation.signals(3).values(:,1);
yaw_estimate = orientation_estimation.signals(3).values(:,2);

figure(4);
hold on; grid on;
plot(time(12000:16500) - 119.99,pitch(12000:16500),time(12000:16500) - 119.99,pitch_estimate(12000:16500));
title('Pitch - za ma³a czêstotliwoœæ próbkowania filtra');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

figure(5);
hold on; grid on;
plot(time(17500:22000) - 174.99,roll(17500:22000),time(17500:22000) - 174.99,roll_estimate(17500:22000));
title('Roll - za ma³a czêstotliwoœæ próbowania filtra');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

figure(6);
hold on; grid on;
plot(time(12000:16500)- 119.99,yaw(12000:16500),time(12000:16500) - 119.99,yaw_estimate(12000:16500));
title('Yaw - za ma³a czêstotliwoœæ próbkowania filtra');
xlabel('Czas [s]');
ylabel('K¹t [deg]');
legend('stan', 'estymata stanu');
hold off;

