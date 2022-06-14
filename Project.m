clc; close all;

syms a1 a2 d2 t1 t2 d3

% need to change paramters such as the height
a1 = 3.82; a2 = 3.82; d2 = 3; 
L1 = Link('revolute','d', 0, 'a', 0,'alpha', 0, 'modified'); 
L2 = Link('revolute','d', d2, 'a', a1,'alpha', 0, 'modified');
L3 = Link('prismatic','theta', 0, 'a', a2, 'alpha', 0, 'modified');
% make the robot
tool = transl(0, 0, -3);
Ryan = SerialLink([L1 L2 L3], 'name', 'Ryan', 'tool', tool);

% th = [t1 t2 d3];
th = [pi/2 pi 2];

% forward kinematics
FK = Ryan.fkine(th);

th1 = [1 0 0 3; 0 1 0 -3; 0 0 1 2; 0 0 0 1];
th2 = [1 0 0 3; 0 1 0 -4; 0 0 1 2; 0 0 0 1];


Traj = ctraj(th1,th2,10); %same number
 for i = 1:10
    temp = Ryan.ikine(Traj(:,:,i),'mask',[1 1 1 0 0 0])*180/pi;
    theta1(i) = temp(1);
    theta2(i) = temp(2);
 end
 
thetas = [theta1; theta2]'
 
% inverse kinematics
IK = Ryan.ikine(FK,'mask',[1 1 1 0 0 0]);

%%
 for j = 1:10 %same number
    qt3(j,:) = Ryan.ikine(Traj(:,:,j),'mask',[1 1 1 0 0 0])
    Ryan.plot(qt3(j,:),'workspace',[-10 10 -10 10 -5 10]);
    [rotation, position] = tr2rt(Traj(:,:,j));
    plot3(position(1),position(2),position(3),'r*')
    hold on    
 end
%% plot robot
figure();
Ryan.plot([0 0 0],'workspace',[-10 10 -10 10 -5 10]);
%% Monte Carlo Simulation
for t1 = .01:0.05:2*pi
    for t2 = .01:0.05:2*pi
        forward = Ryan.fkine([t1 t2 0]);
        [R0T, P0T] = tr2rt(forward);
        plot(P0T(1),P0T(2),'r.');
        hold on
    end
end
hold on
rectangle('Position',[-2.95 -5.22 5.9 5.9],'FaceColor','w','Curvature',0.2)
xlabel('X Position (in)')
ylabel('Y Position (in)')
title('Workspace of a 2D Planar Robot')
axis equal;