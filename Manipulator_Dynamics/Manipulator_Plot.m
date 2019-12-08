function [h,k] = Manipulator_Plot(varargin)
q0=(varargin{1})';
q0d=(varargin{2})';
t_end=varargin{3};
typeOfJoint=varargin{4};
Gear_ratio = varargin{5};       %[kr1,kr2 ...];
Mass_Link = varargin{6};        %[ml1,ml2 ...];
Inertia_Link = varargin{7};    %[Il1,Il2,...];
Inertia_Motor = varargin{8}; %[Im1,Im2,...];
d=varargin{9};
a=varargin{10};
alpha=varargin{11};
theta=varargin{12};
for i= 1:length(Gear_ratio)
    if typeOfJoint=='R'
        L(i)=Link('d',d(i),'a',a(i),'alpha',alpha(i),'m',Mass_Link(i),'r',Position_of_link,'G',Gear_ratio(i),'I',Inertia_Link(i)*eye(3,3),'Jm',Inertia_Motor(i));
    elseif typeOfJoint=='P'
        L(i)=Link('theta',theta(i),'a',a(i),'alpha',alpha(i),'m',Mass_Link(i),'r',Position_of_link,'G',Gear_ratio(i),'I',Inertia_Link(i)*eye(3,3),'Jm',Inertia_Motor(i));
%         L.qlim=[theta(i) d(i)];
    end
end
Robot=SerialLink(L)
[ti,q,qd]=Robot.fdyn(t_end,@torq,q0,q0d);
q=rad2deg(q);
figure('Name','Joint Angles');
h=plot(ti,q)
legend(h);
figure('Name','Joint Rates');
k=plot(ti,qd)
legend(k);