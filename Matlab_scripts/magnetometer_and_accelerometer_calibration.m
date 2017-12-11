clear all;
close all;

load('workspaces\mag_data.mat')
load('workspaces\accelerometer_calibration_data.mat')

% beta = 0.01;
% LB = [-5; -5; -5; -15; -15; -15]; UB = [5; 5; 5; 15; 15; 15];
% bx = 0; by = 0; bz = 0; kx = 0; ky = 0; kz = 0;
% options = optimoptions('lsqnonlin','Display','iter-detailed','TolFun',1e-16,'TolX',1e-16);
% xopt = lsqnonlin('quality_indicator',[kx, ky, kz]',LB, UB,options,time,[mag_x'; mag_y'; mag_z']);

% values taken from experiment not from optimization
offset_mag = [3.48; -0.162; 0.177];
offset_acc = [0.004; 0; 0.021];
gain_mag = [1.17; 1.133; 1.14];
gain_acc = [0.975; 0.99; 0.97];

% ideal magnetometer elipsoid 
x_mag = zeros(80,1); y_mag = x_mag; z_mag = x_mag;
i = 1;
for fi = 0 : pi/40 : 2*pi
    for theta = 0 : pi/80 : pi
        x_mag(i) = 0.47 * cos(fi) * sin(theta);
        y_mag(i) = 0.47 * sin(fi) * sin(theta);
        z_mag(i) = 0.47 * cos(theta);
        i = i + 1;
    end 
end

% ideal accelerometer elipsoid 
x_acc = zeros(80,1); y_acc = x_acc; z_acc = x_acc;
i = 1;
for fi = 0 : pi/40 : 2*pi
    for theta = 0 : pi/80 : pi
        x_acc(i) = cos(fi) * sin(theta);
        y_acc(i) = sin(fi) * sin(theta);
        z_acc(i) = cos(theta);
        i = i + 1;
    end 
end

% data is in gauss, where 1 Gauss is 100uT
figure(1);
plot3((mag_x - offset_mag(1)) * gain_mag(1),(mag_y - offset_mag(2)) * gain_mag(2),(mag_z - offset_mag(3)) * gain_mag(3));
hold on;
plot3(x_mag,y_mag,z_mag);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
grid on;
hold off;

figure(2);
hold on; grid on;
plot3((acc_x_g - offset_acc(1)) * gain_acc(1), (acc_y_g - offset_acc(2)) * gain_acc(2), (acc_z_g - offset_acc(3)) * gain_acc(3));
plot3(x_acc,y_acc,z_acc);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
hold off;


