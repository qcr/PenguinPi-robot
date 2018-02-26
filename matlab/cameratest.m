close all; clear all; clc;

%% Connect to the Rasp Pi
% Get you Pi's IP (type hostname -I into Pi terminal)
% IP = '172.19.226.67';
% IP = 'AlisterCameronPi';
IP = '192.168.1.142';
pb = PiBot(IP);

%% Get Image from Camera
% img = pb.getImageFromCamera();
% imshow(img)
timeout = 0;

testing = 1;
iter = 1;
time = zeros(1000000,1);
while(testing)
    tic;
    img2 = pb.getImageFromCamera();
    toc
    time(iter) = toc;
    iter = iter+1;
    if toc > 0.1
        timeout = timeout + 1;
    end
    if iter>=150
        break;
    end 
end 

id = find(time==0);
time(id) = [];
histogram(time,20)

