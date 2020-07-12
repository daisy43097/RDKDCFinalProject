% Script for ifnal project using gradient-based descent
%

%%Instantiations
Rd = [0 0 1; 0 1 0; -1 0 0];
vd = [0; 0; 0.200]; % m

qI = @(t1) [t1;-pi/2 + 0.1;pi/3 + 0.1;-pi/3 + 0.1;-pi/4 + 0.1;0];

K = 1.8;

ur5 = ur5_interface();
goalFrame = tf_frame('base_link','goal',eye(4));

offset = [-pi; -pi/2; 0; -pi/2; 0; 0];
ur5.move_joints([-pi/2 + 0.1;-pi/2 + 0.1;pi/3 + 0.1;-pi/3 + 0.1;-pi/4 + 0.1;0],5);

%% Take in input locations

pd1 = input("Please enter the first desired position as an array (i.e. [1 2 3])> ");
% pd2 = input("Please enter the second desired position as an array (i.e. [1 2 3])> ");
pd1 = [0.5000   -0.5000         0];
% disp(pd1); disp(pd2);

%% Set up goal positions
gd = [Rd pd1'; 0 0 0 1];
gI = [Rd pd1'+vd; 0 0 0 1];
goalFrame.move_frame('base_link',gI);


%% Align robot with target (gesture)
t1 = atan2(pd1(2),pd1(1));
ur5.move_joints(qI(t1), 5);



%% move to intermediate pose
goalFrame.move_frame('base_link',gI);
result = ur5gradientdescent(gI, K, ur5)


%% move to goal pose
goalFrame.move_frame('base_link',gd);
result = ur5gradientdescent(gd, K, ur5)
