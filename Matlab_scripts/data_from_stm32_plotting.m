close all;
clear all;

load('workspaces\stm32_accelerometer_all_axis_data3.mat')

time = time * 1000;

figure(1);
hold on; grid on;
plot(time(6976:end) - 5.999,pitch(6976:end),time(6976:end) - 5.999,pitch_estimate(6976:end));
title('Pitch');
legend('stan', 'estymata stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(2);
hold on; grid on;
plot(time(6976:23557) - 5.999,roll(6976:23557),time(6976:23557) - 5.999,roll_estimate(6976:23557));
title('Roll');
legend('stan', 'estymata stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(3);
hold on; grid on;
plot(time(6976:end) - 5.999,yaw(6976:end),time(6976:end) - 5.999,yaw_estimate(6976:end));
title('Yaw');
legend('stan', 'estymata stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;