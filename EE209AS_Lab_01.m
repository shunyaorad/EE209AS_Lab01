%% EE209AS Lab 01
%  Author:Shun Yao
%% Inverse Kinematics Evaluation
clc
clear all

% link lengths
l1 = 0;
l2 = 6;
l3 = 12;
l4 = 12;
l5 = 12;

% link angles
theta_1 = pi/3;
theta_2 = pi/3;
theta_3 = pi/3;
theta_4 = pi/3;
theta_5 = 0;
theta = [theta_1, theta_2, theta_3, theta_4, theta_5];

% generate desired coordinate using selected link angles
desired_coordinate = forwardKinematics(theta)

% generate link angles derived by Inverse Kinematics
result_angle = inverseKinematics(desired_coordinate)

% generate position using derived angles using ForwardKinematics
result_position = forwardKinematics(result_angle)

% norm of error between the desired and generated position
error_norm = norm(result_position-desired_coordinate)

%% Trajetory tracking evaluation

target_trajectory = [linspace(11,0,10);linspace(0,-3,10);linspace(25,28,10)]; 

% generate link angles using Inverse Kinematics
for i=1:length(target_trajectory)
    target = target_trajectory(:,i);
    IKangle = inverseKinematics(target);
    IKangles(:,i) = IKangle;
end

% Calculate resulting trajectory using derived link angles
trajectory = forwardKinematics(IKangles(:,1));
for i=1:length(IKangles)
    input = IKangles(:,i);
    point = forwardKinematics(input);
    trajectory(:,i) = point;
end

%% Plot
fX = target_trajectory(1,:);
fY = target_trajectory(2,:);
fZ = target_trajectory(3,:);
figure(1)
plot3(fX,fY,fZ)
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on

IX = trajectory(1,:);
lY = trajectory(2,:);
lZ = trajectory(3,:);
plot3(IX,lY,lZ,'r')
xlabel('X')
ylabel('Y')
zlabel('Z')

% trajectory start point
x0 = double(IX(1));
y0 = double(lY(1));
z0 = double(lZ(1));

% trajectory end point
xe = double(IX(end));
ye = double(lY(end));
ze = double(lZ(end));

t = text(x0,y0,z0,'start point');
s = t.FontSize;
t.FontSize = 12;
t2 = text(xe,ye,ze,'end point');
legend('Target trajectory','Generated Trajectory using Inverse Kinematics','Location', 'northeast')
grid on
hold off

% plot joint angles
theta1 = IKangles(1,:);
theta2 = IKangles(2,:);
theta3 = IKangles(3,:);
theta4 = IKangles(4,:);

% plot link angles
subplot(4,1,1)
plot(theta1)
ylabel('radian')
title('theta 1')
subplot(4,1,2)
plot(theta2)
ylabel('radian')
title('theta 2')
subplot(4,1,3)
plot(theta3)
ylabel('radian')
title('theta 3')
subplot(4,1,4)
plot(theta4)
ylabel('radian')
title('theta 4')

%% Trajetory evaluation using end points, linear interpolation
clc
clear all

% link lengths
l1 = 0;
l2 = 6;
l3 = 12;
l4 = 12;
l5 = 12;

% Target position
target = [11;10;25];

% generate link angles derived by Inverse Kinematics
result_trajectory = inverseKinematicsPlot(target);

linearTrajectory = [linspace(30,11,20); linspace(0,10,20); (linspace(0,25,20))];

%% Plot

figure()
rX = result_trajectory(1,:);
rY = result_trajectory(2,:);
rZ = result_trajectory(3,:);
figure(1)
plot3(rX,rY,rZ)
xlabel('X(cm)')
ylabel('Y(cm)')
zlabel('Z(cm)')
hold on

lX = linearTrajectory(1,:);
lY = linearTrajectory(2,:);
lZ = linearTrajectory(3,:);
plot3(lX,lY,lZ,'r')

x0 = double(rX(1));
y0 = double(rY(1));
z0 = double(rZ(1));
xe = double(lX(end));
ye = double(lY(end));
ze = double(lZ(end));


t = text(x0,y0,z0,'start point');
s = t.FontSize;
t.FontSize = 12;
t2 = text(xe,ye,ze,'end point');
legend('Trajectory of robot using inverse kinematics','linear trajectory between endpoints','Location', 'northeast')
grid on
hold off

