% square

url = "http://10.0.0.20";
robot = PiBot(url)

% 5.33V mm/sec
% to go 500 mm at speed 30 is
V = 30
t = 500/5.33/V



% to turn 90 deg  T omega = pi/2 = dV/W, T dV/W = pi/2, T dV = Wpi/2
% to turn in 1 second
W = 0.156;
dV = W*pi/2   % m/s
Tturn = 1;
dV = dV*1000/5.33

sh = robot.stop()

for i=1:4
    s = robot.setVelocity([V V], t);
    sh = [sh s];
    s = robot.setVelocity([dV -dV]/2, Tturn);
    sh = [sh s];
end