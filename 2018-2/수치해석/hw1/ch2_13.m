clear
close all
clc
v = 10:10:80;
F = [25 70 380 550 610 1220 830 1450];
vf = 0:100;
Ff = 0.2741*vf.^1.9842;
plot(v,F,'om-',vf,Ff,'-.k')
xlabel('v')
ylabel('F');