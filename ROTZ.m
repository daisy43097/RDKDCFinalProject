function ROTZ = ROTZ(theta)
% function accepts a angle in rad and returns the 3x3 rotation matrix about
% z axis
[rows, cols] = size(theta);
if rows~= 1 | cols ~= 1
    error('input must be a scalar');
else 
    ROTZ = [cos(theta) -sin(theta) 0;
            sin(theta) cos(theta) 0;
                0 0 1];
end 