clear all;
close all;

q = 0.0001;r = 1;
Q = [0.8 0;0 0.1]*q;   % niepewnoœæ procesu (szumy procesowe i niepewnoœæ modelowania)
R = r;                 % niepewnoœæ pomiaru (im mniejsze tym bardziej ufamy pomiarom)
        
dt = 0.001;
A = [1 -dt;0 1];
B = [dt; 0];
C = [1 0];

Pk_corr_0 = Q; 

mag_x_offset = 3.48;
mag_y_offset = -0.162;
mag_z_offset = 0.177;

mag_x_gain = 1.17;
mag_y_gain = 1.133;
mag_z_gain = 1.14;

acc_x_offset = 0.004;
acc_y_offset = 0;
acc_z_offset = 0.021;

acc_x_gain = 0.975;
acc_y_gain = 0.99;
acc_z_gain = 0.97;