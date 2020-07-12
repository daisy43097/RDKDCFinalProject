function ROTX = ROTX(theta)
% function accepts a angle in rad and returns the 3x3 rotation matrix about
% x axis

[rows, cols] = size(theta);
if rows ~= 1 || cols ~= 1
    error('Input must be a scalar');
else 
    ROTX = [1 0 0;
         0 cos(theta) -sin(theta);
         0 sin(theta) cos(theta)];
end 
