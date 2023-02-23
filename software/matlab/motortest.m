close all; clear all; clc;
%% Run when 

%% Connect to the Rasp Pi
% Get you Pi's IP (type hostname -I into Pi terminal or lookup on DHCP Table from router)
IP = '131.181.33.163';
pb = PiBot(IP);

%% Set Motor Speeds
disp('Setting Motor Speeds...')

disp('Motor A and B at -50,-50 speeds')
pb.setVelocity(-50,-50);
pause(1);

disp('Motor B at 50 and Motor A off')
pb.setVelocity(0,50);
pause(1);

pb.setVelocity(0,0);

%% Get Encoder Ticks
disp('Getting Encoder Ticks...')
[ticks_left, ticks_right] = pb.getEncoders();

fprintf('The value of ticks_left is %d\n', ticks_left);
fprintf('The value of ticks_right is %d\n', ticks_right);

pb.resetEncoder();
