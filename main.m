% MAE C163A/C263A Project
% Team X
clear;clc;
sync_read_write();
%% Close Port
closePort(port_num);
clc;

%%

h = 20;
current_time = 0;
tic

syms a1 a2 d2 t1 t2 d3
% need to change paramters such as the height
a1 = 3.82; a2 = 3.621; d2 = 3;
L1 = Link('revolute','d', 0, 'a', 0,'alpha', 0, 'modified');
L2 = Link('revolute','d', d2, 'a', a1,'alpha', 0, 'modified');
L3 = Link('prismatic','theta', 0, 'a', a2, 'alpha', 0, 'modified');
% make the robot
tool = transl(0, 0, -3);
Ryan = SerialLink([L1 L2 L3], 'name', 'Ryan', 'tool', tool);

% via point for each point (p is on paper, v is above)
height = 3.25;
down = 1.5;
v1 = traj_pt(3,2.5,height);
p1 = traj_pt(3,2.5,down);
p2 = traj_pt(3,1,down);
v2 = traj_pt(3,1,height);
v3 = traj_pt(3,0,height);
p3 = traj_pt(3,0,down);
p4 = traj_pt(3,-1.5,down);
v4 = traj_pt(3,-1.5,height);
v5 = traj_pt(4.5,-1.5,height);
p5 = traj_pt(4.5,-1.5,down);
p6 = traj_pt(4.5,0,down);
v6 = traj_pt(4.5,0,height);
v7 = traj_pt(4.5,1,height);
p7 = traj_pt(4.5,1,down);
p8 = traj_pt(4.5,2.5,down);
v8 = traj_pt(4.5,2.5,height);
v9 = traj_pt(6,2.5,height);
p9 = traj_pt(6,2.5,down);
p10 = traj_pt(6,1,down);
v10 = traj_pt(6,1,height);
v11 = traj_pt(6,0,height);
p11 = traj_pt(6,0,down);
p12 = traj_pt(6,-1.5,down);
v12 = traj_pt(6,-1.5,height);
v13 = v11;
p13 = p11;
p14 = p7;
v14 = v7;
v15 = v2;
p15 = p2;
p16 = p6;
v16 = v6;
v17 = v12;
p17 = p12;
p18 = traj_pt(4.5,-2.75,down);
p19 = p4;
v19 = v4;
v20 = v1;
p20 = p1;
p21 = traj_pt(4.5,3.75,down);
p22 = p9;
v22 = v9;
v23 = v10;
p23 = p10;
p24 = p7;
v24 = v7;
v25 = v6;
p25 = p6;
p26 = p3;
% move away to see the paper
v26 = v3;
v27 = traj_pt(3,-5,3);

points = cat(1,v1,p1,p2,v2,v3,p3,p4,v4,v5,p5,p6,v6,v7,p7,p8,v8,v9,p9,p10,v10,v11,p11,p12,v12,v13,p13,p14,v14,v15,p15,p16,v16,v17,p17,p18,p19,v19,v20,p20,p21,p22,v22,v23,p23,p24,v24,v25,p25,p26,v26,v27);

% Ctraj
ii = 1;
for i = 1:4:length(points)-7
    Traj{ii} = ctraj(points(i:i+3,:),points(i+4:i+7,:),h);
    ii = ii + 1;
end

for j = 1:length(Traj)
    for i = 1:h
        temp_1 = ik_test(Traj{j}(:,:,i))*(4096/(2*pi));
        theta1(i) = temp_1(1)+2048;
        theta2(i) = temp_1(2)+2048;
        distance(i) = temp_1(3)+2048;
    end
    theta1_new{j} = theta1;
    theta2_new{j} = theta2;
    distance_new{j} = distance;
end

theta1_full = 0;
theta2_full = 0;
distance_full = 0;
Traj_full = diag([0 0 0 0]);

for i = 1:length(Traj)
    theta1_full = cat(2,theta1_full,theta1_new{i});
    theta2_full = cat(2,theta2_full,theta2_new{i});
    distance_full = cat(2,distance_full,distance_new{i});
    for k = 1:h
    Traj_full = cat(1,Traj_full,Traj{i}(:,:,k));
    end
end

theta_list = [theta1_full; theta2_full; distance_full]';
%%
qt3 = theta_list*2*pi/4096;
kk = 1;
for j = 1:length(qt3)
    Ryan.plot(qt3(j,:),'workspace',[-10 10 -10 10 1 2])

    [rot,pos] = tr2rt(Traj_full(kk:kk+3,1:4));
    plot3(pos(1),pos(2),pos(3),'r.')
    kk = kk + 4;
    hold on
end

%% Sync 
    theta1_actual = zeros(1,length(theta_list));
    theta2_actual = zeros(1,length(theta_list));
    theta3_actual = zeros(1,length(theta_list));

    % Add parameter storage for Dynamixel#0 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL0_ID);
    
    % Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL1_ID);

    % Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL2_ID);

for kk = 1:length(theta_list)
    Theta1 = theta_list(kk,1);%+init_pos(1);
    Theta2 = theta_list(kk,2);%+init_pos(2);
    distance3 = theta_list(kk,3); %+init_pos(3);
    

    % Add Dynamixel#0 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL0_ID, typecast(int32(Theta1), 'uint32'), LEN_PRO_GOAL_POSITION);
     
    % Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL1_ID, typecast(int32(Theta2), 'uint32'), LEN_PRO_GOAL_POSITION);
   
    % Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL2_ID, typecast(int32(distance3), 'uint32'), LEN_PRO_GOAL_POSITION);

    % Syncwrite goal position
    groupSyncWriteTxPacket(groupwrite_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
   

    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(groupwrite_num);
    
    % Syncread present position (COMMENT IT OUT WHEN DEMO)
    groupSyncReadTxRxPacket(groupread_num);
    
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);

    dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL0_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
    dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    % Get Dynamixel#0 present position value
    theta1_actual(kk) = groupSyncReadGetData(groupread_num, DXL0_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    % Get Dynamixel#1 present position value
    theta2_actual(kk) = groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    % Get Dynamixel#2 present position value
    theta3_actual(kk) = groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

end

%% Plots

theta1_actual_T = theta1_actual';
theta2_actual_T = theta2_actual';
theta3_actual_T = theta3_actual';
time_actual = linspace(0,25,length(theta1_actual_T));

err1 = rad2deg((theta_list(:,1) - theta1_actual_T)*((2*pi)/4096));
err2 = rad2deg((theta_list(:,2) - theta2_actual_T)*((2*pi)/4096));
err3 = rad2deg((theta_list(:,3) - theta3_actual_T)*((2*pi)/4096));

figure(1)

subplot(3,1,1)
plot(time_actual,err1)
title('Error Analysis of Motor 1')
axis([0 25 -100 100])
% axis([0 25 -10 10])
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')

subplot(3,1,2)
plot(time_actual,err2)
title('Error Analysis of Motor 2')
axis([0 25 -100 100])
% axis([0 25 -10 10])
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')

subplot(3,1,3)
plot(time_actual,err3)
axis([0 25 -100 100])
% axis([0 25 -10 10])
title('Error Analysis of Motor 3')
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')

figure(2)
subplot(3,1,1)
plot(time_actual,rad2deg((theta_list(:,1))*((2*pi)/4096)))
hold on
plot(time_actual,rad2deg((theta1_actual_T)*((2*pi)/4096)))
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')
title('Joint Angles of Motor 1')
legend('Goal Position','Actual Position')

subplot(3,1,2)
plot(time_actual,rad2deg((theta_list(:,2))*((2*pi)/4096)))
hold on
plot(time_actual,rad2deg((theta2_actual_T)*((2*pi)/4096)))
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')
title('Joint Angles of Motor 2')
legend('Goal Position','Actual Position')

subplot(3,1,3)
plot(time_actual,rad2deg((theta_list(:,3))*((2*pi)/4096)))
hold on
plot(time_actual,rad2deg((theta3_actual_T)*((2*pi)/4096)))
xlabel('Time [sec]'); ylabel('Joint Angle [degrees]')
title('Joint Angles of Motor 3')
legend('Goal Position','Actual Position')
%% Function

function A = traj_pt(x,y,z)
    A = [1 0 0 x; 0 1 0 y; 0 0 1 z;0 0 0 1];
end

function t = ik_test(T_g)
    a1 = 3.82; a2 = 3.6210; d2 = 3;
    [rot,pos] = tr2rt(T_g);
    x = pos(1); y = pos(2); z = pos(3);
    d3 = z - d2;
    q2 = acos((x^2+y^2-a1^2-a2^2)/(2*a1*a2));
    q1 = atan2(y,x) - atan2(a2*sin(q2),a1+a2*cos(q2));
    t = [q1 q2 d3];
end

function [theta1,theta2,distance] = traj_pt_gen(a1,a2,d2,h,p1,p2)
    Traj = ctraj(p1,p2,h);
    
    for i = 1:h
        temp = ik_test(a1,a2,d2,Traj(:,:,i))*(4096/(2*pi));
        theta1(i) = temp(1)+2048;
        theta2(i) = temp(2)+2048;
        distance(i) = temp(3)+2048;
    end
end