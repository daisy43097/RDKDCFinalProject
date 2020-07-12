%% UR5 Final Project

% Initialize ur5
ur5 = ur5_interface();

%Initialize calculation parameters
Rd = [-1 0 0; 0 1 0; 0 0 -1];
%Rd = ROTZ(-pi/2)*ROTY(pi/2)*[0 -1 0; 0 0 1; -1 0 0];
vd = [0; 0; 0.200]; % m
offset = [0; 0; 0; -pi/2; -pi/2; 0]; % for Rviz simulation

%% Preperation 
% Input start and target position 
pd1 = input('Please enter the first desired position as an array (i.e. [1 2 3])> ');
pd2 = input('Please enter the second desired position as an array (i.e. [1 2 3])> ');
% pd1 = [0.47 0.55 0.12];
% pd2 = [-0.3 0.39 0.12];


% gst for start and target 
g_start = [Rd pd1'; 0 0 0 1];
g_target = [Rd pd2'; 0 0 0 1];

% gst for above start and above target 
g_startab = [Rd pd1'+vd; 0 0 0 1];
g_targetab = [Rd pd2'+vd; 0 0 0 1];

%% IK calculation

% --------------above start configuration ------------------%
Q_startab_all = ur5InvKin(g_startab);

% collision check 
Q_startab_valid = zeros(6,1);

n = 1;
for n = 1:length(Q_startab_all)
    Q1 = Q_startab_all(:,n);
    % take Q if theta2 is above the table
    if Q1(2) > -pi && Q1(2) < 0
        g_4 = ur5FwdKin_4(Q1); % calculate 4th joint gst 
        % take Q if (4th joint+pen) is above the table
        if g_4(3,4) > 0.05
            Q_startab_valid(:,n) = Q1;
            n = n+1;
        end 
    end 
end 

% find the best Q_startab 
norm_startab_val = zeros(size(Q_startab_valid,2),1);
for j = 1:length(norm_startab_val)
    % find the distance from home position to Q_startab
    norm_startab_val(j) = norm(Q_startab_valid(:,j) - ur5.home);
end 
[val,I] = min(norm_startab_val);
Q_startab = Q_startab_valid(:,I);


% -------------- start configuration ------------------%
Q_start_all = ur5InvKin(g_start);

% collision check 
Q_start_valid = zeros(6,1);

n = 1;
for n = 1:length(Q_start_all)
    Q3 = Q_start_all(:,n);
    % take Q if theta2 is above the table
    if Q3(2) > -pi && Q3(2) < 0
        g_4 = ur5FwdKin_4(Q3); % calculate 4th joint gst 
        % take Q if (4th joint+pen) is above the table
        if g_4(3,4) > 0.05 %%%%%%%%%%%%%%%
            Q_start_valid(:,n) = Q3;
            n = n+1;
        end 
    end 
end 

% find the best Q_start 
norm_start_val = zeros(size(Q_start_valid,2),1);
for j = 1:length(norm_start_val)
    % find the distance from Q_startab to Q_start
    norm_start_val(j) = norm(Q_start_valid(:,j) - Q_startab);
end 
[val,I] = min(norm_start_val);
Q_start = Q_start_valid(:,I);


% ----------------------above target configuration ------------------%
Q_targetab_all = ur5InvKin(g_targetab);

% collision check
Q_targetab_valid = zeros(6,1);

n = 1;
for n = 1:length(Q_targetab_all)
    Q2 = Q_targetab_all(:,n);
    % take Q if theta2 is above the table
    if Q2(2) > -pi && Q2(2) < 0
        g_4 = ur5FwdKin_4(Q2); % calculate 4th joint gst 
        % take Q if (4th joint+pen) is above the table
        if g_4(3,4) > 0.05
            Q_targetab_valid(:,n) = Q2;
            n = n+1;
        end 
    end 
end 

% find the best Q_targetab  
norm_targetab_val = zeros(size(Q_targetab_valid,2),1);
for j = 1:length(norm_targetab_val)
    % find the distance from home position to Q_targetab
    norm_targetab_val(j) = norm(Q_targetab_valid(:,j) - Q_startab);
end 
% find the minimum distance from home to Q_startab and take the
% configuration
[value,K] = min(norm_targetab_val);
Q_targetab = Q_targetab_valid(:,K);


% ---------------------- target configuration ------------------%
Q_target_all = ur5InvKin(g_target);

% collision check
Q_target_valid = zeros(6,1);

n = 1;
for n = 1:length(Q_target_all)
    Q4 = Q_target_all(:,n);
    % take Q if theta2 is above the table
    if Q4(2) > -pi && Q4(2) < 0
        g_4 = ur5FwdKin_4(Q4); % calculate 4th joint gst 
        % take Q if (4th joint+pen) is above the table
        if g_4(3,4) > 0.05%%%%%%%%%%%%%%%%%%%%%%%%%
            Q_target_valid(:,n) = Q4;
            n = n+1;
        end 
    end 
end 

% find the best Q_target  
norm_target_val = zeros(size(Q_target_valid,2),1);
for j = 1:length(norm_target_val)
    % find the distance from Q_targetab to Q_target
    norm_target_val(j) = norm(Q_target_valid(:,j) - Q_targetab);
end 
% find the minimum distance from Q_targetab to Q_target and take the
% configuration
[value,K] = min(norm_target_val);
Q_target = Q_target_valid(:,K);

%% IK control

% home position
ur5.move_joints(ur5.home,10)
pause(10)
% above start position 
ur5.move_joints(Q_startab,10);
pause(10)
% start position 
ur5.move_joints(Q_start,10);
pause(10)
% above start position 
ur5.move_joints(Q_startab,10);
pause(10)
% above target position 
ur5.move_joints(Q_targetab,10);
pause(10)
% target position 
ur5.move_joints(Q_target,10);
pause(10)
% above target position 
ur5.move_joints(Q_targetab,10);
pause(10)
% home position
ur5.move_joints(ur5.home,10)
pause(10)

