close all;
clear all;

% experiment 1
load('workspaces\stm32_accelerometer_all_axis_data1.mat')

figure(1);
hold on; grid on;
plot(time,pitch,time,pitch_estimate);
title('Pitch');
legend('stan', 'estymata_stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(2);
hold on; grid on;
plot(time,roll,time,roll_estimate);
title('Roll');
legend('stan', 'estymata_stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;

figure(3);
hold on; grid on;
plot(time,yaw,time,yaw_estimate);
title('Yaw');
legend('stan', 'estymata_stanu')
xlabel('Czas [s]');
ylabel('K¹t [deg]');
hold off;