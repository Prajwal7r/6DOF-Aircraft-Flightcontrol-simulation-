% initialiing x0 constants for integrator 

clear 
clc
close all
%  defining constants 
load trim_values.mat
x0 = xstar;
u = ustar;

x0 = [85;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0];

v0 = [0;
    0;
    0];
% 
u = [0;
    -0.1;  % -6 degree 
     0.05; 
     0.08;      %right wing throttle 
     0.08];     % left wing throttle


T = 1000;

% -------------control limits/saturations-----------------------

u1min = -25*pi/180;
u1max =  25*pi/180;

u2min = -25*pi/180;
u2max = 10*pi/180;

u3min = -30*pi/180;
u3max = 30*pi/180;

u4min = 0;
u4max = 10*pi/180;

u5min = 0;
u5max = 10*pi/180;

% RCAM_model(x0 , u)
out = sim("RCAM.slx")
t = out.simx.Time;

u1 =  out.xout.Data(:,1);
u2 = out.xout.Data(:,2);
u3 = out.xout.Data(:,3);
u4 = out.xout.Data(:,4);
u5 = out.xout.Data(:,5);

x1 = out.simx.Data(:,1);
x2 = out.simx.Data(:,2);
x3 = out.simx.Data(:,3);
x4 = out.simx.Data(:,4);
x5 = out.simx.Data(:,5);
x6 = out.simx.Data(:,6);
x7 = out.simx.Data(:,7);
x8 = out.simx.Data(:,8);
x9 = out.simx.Data(:,9);

figure 
subplot(5,1,1)
plot(t,u1)
legend('u_1')
grid on 

subplot(5,1,2)
plot(t,u2)
legend('u_2')
grid on 

subplot(5,1,3)
plot(t,u3)
legend('u_3')
grid on 

subplot(5,1,4)
plot(t,u4)
legend('u_4')
grid on 

subplot(5,1,5)
plot(t,u5)
legend('u_5')
grid on 


% ploting state vectors 

figure 
subplot(3,3,1)
plot(t,x1,'MarkerSize',3)
legend('u')
grid on 

subplot(3,3,4)
plot(t,x2)
legend('v')
grid on 

subplot(3,3,7)
plot(t,x3)
legend('w')
grid on 

% p q r 

subplot(3,3,2)
plot(t,x4)
legend('roll rate')
grid on 

subplot(3,3,5)
plot(t,x5)
legend('pitch rate')
grid on 

subplot(3,3,8)
plot(t,x6)
legend('yaw rate')
grid on 

subplot(3,3,3)
plot(t,x7)
legend('\phi roll angle')
grid on 

subplot(3,3,6)
plot(t,x8)
legend('\theta pitch angle')
grid on 

subplot(3,3,9)
plot(t,x9)
legend('\psi yaw angle')
grid on 
%%
figure()
plot(t,x9,'LineWidth',2)
legend('\psi yaw angle')
grid on 

%%
figure()
plot(t,x8,'LineWidth',2)
legend('\theta pitch angle')
grid on 
%%
figure()
plot(t,x7,'LineWidth',2)
legend('\phi roll angle')
grid on 


