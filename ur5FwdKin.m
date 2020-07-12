
function gst = ur5FwdKin(q)
% Input q is a 6x1 joint space variable vector 

[rows, cols] = size(q);
if rows ~= 6 || cols ~= 1
    error( 'the input requires a 6x1 vector')
end 
 L0 = 89.2/1000;
 L1 = 425/1000;
 L2 = 392/1000;
 L3 = 109.3/1000;
 L4 = 94.75/1000;
 L5 = 82.5/1000;
 % gst0 is the zero configuration of UR5
 gst0 = [eye(3) [L3+L5;0;L1+L2+L4+L0]; 0 0 0 1];
 %[1 0 0 L3+L5; 0 1 0 0; 0 0 1 L1+L2+L4+L0; 0 0 0 1]
 
 %twist for each joint 
 twist1 = [0 -1 0 0; 1 0 0 0;0 0 0 0;0 0 0 0];
 twist2 = [0 0 0 0; 0 0 -1 L0; 0 1 0 0; 0 0 0 0];
 twist3 = [0 0 0 0; 0 0 -1 L1 + L0; 0 1 0 0; 0 0 0 0];
 twist4 = [0 0 0 0; 0 0 -1 L1+L2 + L0; 0 1 0 0; 0 0 0 0];
 twist5 = [0 -1 0 0; 1 0 0 -L3; 0 0 0 0; 0 0 0 0];
 twist6 = [0 0 0 0; 0 0 -1 L1+L2+L4+L0; 0 1 0 0; 0 0 0 0];
 
 newthi1 = twist1*q(1);
 newthi2 = twist2*q(2);
 newthi3 = twist3*q(3);
 newthi4 = twist4*q(4);
 newthi5 = twist5*q(5);
 newthi6 = twist6*q(6);
 
 %gst is the end effector pose. it should return a 4x4 matrix 
 gst = expm(newthi1)*expm(newthi2)*expm(newthi3)*expm(newthi4)*expm(newthi5)*expm(newthi6)*gst0;
 
end