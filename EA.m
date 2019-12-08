function [RA] = EA(varargin)
if nargin <2
    pm_error('More inputs needed')
else
    F = varargin{1};I = varargin{2};
    if isequal(I,'Angles')
        angle = varargin{3};
        angles=deg2rad(angle);
        if isequal(F,'Current frame')
            R = [cos(angles(1))*cos(angles(2))*cos(angles(3))-sin(angles(1))*sin(angles(3)) -cos(angles(1))*cos(angles(2))*sin(angles(3))-sin(angles(1))*cos(angles(3)) cos(angles(1))*sin(angles(2)); 
                sin(angles(1))*cos(angles(2))*cos(angles(3))+cos(angles(1))*sin(angles(3)) -sin(angles(1))*cos(angles(2))*sin(angles(3))+cos(angles(1))*cos(angles(3)) sin(angles(1))*sin(angles(2));
                -sin(angles(2))*cos(angles(3)) sin(angles(2))*sin(angles(3)) cos(angles(2))];
        elseif isequal(F,'Fixed frame')
           R = [cos(angles(1))*cos(angles(2)) cos(angles(1))*sin(angles(2))*sin(angles(3))-sin(angles(1))*cos(angles(3)) cos(angles(1))*sin(angles(2))*cos(angles(3))+sin(angles(1))*sin(angles(3)) ; 
                sin(angles(1))*cos(angles(2)) sin(angles(1))*sin(angles(2))*sin(angles(3))+cos(angles(1))*cos(angles(3)) sin(angles(1))*sin(angles(2))*cos(angles(3))-cos(angles(1))*sin(angles(3)) ;
                -sin(angles(2)) cos(angles(2))*sin(angles(3)) cos(angles(2))*cos(angles(3))];
        end
        RA = R;
    elseif isequal(I,'Rotation matrix')
        R = varargin{3};
        if isequal(F,'Current frame')
            a1 = atan2(R(2,3),R(1,3));
            a2 = atan2(-R(2,3),-R(1,3));
            b1 = atan2(sqrt(R(1,3)^2+R(2,3)^2),R(3,3));
            b2 = atan2(-sqrt(R(1,3)^2+R(2,3)^2),R(3,3));
            c1 = atan2(R(3,2),-R(3,1));
            c2 = atan2(-R(3,2),R(3,1));
        elseif isequal(F,'Fixed frame')
            a1 = atan2(R(2,1),R(1,1));
            a2 = atan2(-R(2,1),-R(1,1));
            b1 = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
            b2 = atan2(-R(3,1),-sqrt(R(3,2)^2+R(3,3)^2));
            c1 = atan2(R(3,2),R(3,3));
            c2 = atan2(-R(3,2),-R(3,3));
        end
        RA = [a1 b1 c1;a2 b2 c2];
    end
end
