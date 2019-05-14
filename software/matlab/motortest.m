close all; clear all; clc;
%% Run when 

%% Connect to the Rasp Pi
% Get you Pi's IP (type hostname -I into Pi terminal or lookup on DHCP Table from router)
IP = '131.181.33.181';
pb = PiBot(IP);

%% Set Motor Speeds
disp('Setting Motor Speeds...')

disp('Motor A and B at -50,-50 speeds')
pb.setMotorSpeeds(-50,-50);
pause(1);

disp('Motor B at 50 and Motor A off')
pb.setMotorSpeeds(0,50);

%% Get Encoder Ticks
disp('Getting Encoder Ticks...')
ticks = pb.getMotorTicks(); disp(ticks)
disp('1st Number for Motor A, then B and so on...')

pb.setMotorSpeeds(0,0);