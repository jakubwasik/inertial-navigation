close all;
clear all;

LSM6DS33_ACC_RESOLUTION = 2.0;
LSM6DS33_GYRO_RESOLUTION = 245.0;

s = serial('COM4');
set(s, 'BaudRate', 115200);
fopen(s);

time = 0 : 0.1 : 10;
tspan = length(time);
acc_x = zeros(1,tspan); acc_y = zeros(1,tspan); acc_z = zeros(1,tspan);
gyro_x = zeros(1,tspan); gyro_y = zeros(1,tspan); gyro_z = zeros(1,tspan);

for i = 1 : tspan
    disp('Iteration number = ');
    disp(i);
    data = fread(s, 12);

    % decode the data
    temp_gyro_x = bitsll(int16(data(2)),8) + int16(data(1));
    temp_gyro_y = bitsll(int16(data(4)),8) + int16(data(3));
    temp_gyro_z = bitsll(int16(data(6)),8) + int16(data(5));
    temp_acc_x = bitsll(int16(data(8)),8) + int16(data(7));
    temp_acc_y = bitsll(int16(data(10)),8) + int16(data(9));
    temp_acc_z = bitsll(int16(data(12)),8) + int16(data(11));

    % convert data to signed 16-bit integer
    gyro_x(i) = convert_to_signed_int16(temp_gyro_x);
    gyro_y(i) = convert_to_signed_int16(temp_gyro_y);
    gyro_z(i) = convert_to_signed_int16(temp_gyro_z);
    acc_x(i) = convert_to_signed_int16(temp_acc_x);
    acc_y(i) = convert_to_signed_int16(temp_acc_y);
    acc_z(i) = convert_to_signed_int16(temp_acc_z);
    
    pause(0.1);

end

acc_x = double(acc_x * LSM6DS33_ACC_RESOLUTION) / double(intmax('int16'));
acc_y = double(acc_y * LSM6DS33_ACC_RESOLUTION) / double(intmax('int16'));
acc_z = double(acc_z * LSM6DS33_ACC_RESOLUTION) / double(intmax('int16'));

gyro_x = double(gyro_x * LSM6DS33_GYRO_RESOLUTION) / double(intmax('int16'));
gyro_y = double(gyro_y * LSM6DS33_GYRO_RESOLUTION) / double(intmax('int16'));
gyro_z = double(gyro_z * LSM6DS33_GYRO_RESOLUTION) / double(intmax('int16'));

figure(1);
hold on; grid on;
plot(time, acc_x, time, acc_y, time, acc_z);
title('Linear acceleration X,Y,Z axis');
legend('X axis','Y axis','Z axis');
xlabel('time [s]');
ylabel('Acceleration [g]');
hold off;

figure(2);
hold on; grid on;
plot(time, gyro_x, time, gyro_y, time, gyro_z);
title('Angular rate X,Y,Z axis');
legend('X axis','Y axis','Z axis');
xlabel('time [s]');
ylabel('Angular rate [dps]');
hold off;

fclose(s);