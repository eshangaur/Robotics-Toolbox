function [B,C,g] = Manipulator_Dynamics_2(varargin)
n = varargin{1};
q= varargin{2};
%q = Joint_C();  %q = [q1 q2 ...];
Position_of_link = varargin{3};  % 3xn position of the center of mass of the link
Position_of_motor = varargin{4}; % 3xn position of the motor
Zcf_b = varargin{5}; % with respect to reference frame
link_type = varargin{6};
Gear_ratio = varargin{7};       %[kr1,kr2 ...];
Mass_Link = varargin{8};        %[ml1,ml2 ...];
Mass_Motor = varargin{9};       %[mm1,mm2,...];
Inertia_Link = varargin{10};    %[Il1,Il2,...];
Inertia_Motor = varargin{11};   %[Im1,Im2,...];
g0 = varargin{12};
dq = varargin{13};
for i = 1:n
    for j = 1:n
        dL(:,j) = diff(Position_of_link(:,i),q(j));
        dM(:,j) = diff(Position_of_motor(:,i),q(j));
    end
    JpL{i} = dL;
    JpM{i} = dM;
%     JoL{i} = oL;
%     JoM{i} = oM;
end
for i = 1:n
   for j = 1:n
       if j>i
           J1(:,j) = zeros(3,1);
       elseif j==i
           if strcmp(link_type(j),'R')
               J1(:,j) = Zcf_b(:,j);
           else
               J1(:,j) = zeros(3,1);
           end
       else
           if i==1 && j==1
               J1(:,j)=Zcf_b(:,i);
           else
               jj = JoL{i-1};
               J1(:,j) = jj(:,j);
           end
       end
       JoL{i} = J1;
   end
end
for i = 1:n
   for j = 1:n
       if j>i
           J2(:,j) = zeros(3,1);
       elseif j==i
           a = Zcf_b(:,j);
           J2(:,j) = Gear_ratio(i)*a;
       else
           jj = JoL{i};
           J2(:,j) = jj(:,j);
       end
       JoM{i} = J2;
   end
end

B = 0;

for k = 1:n
    B = B + Mass_Link(k)*(JpL{k}'*JpL{k}) + Mass_Motor(k)*(JpM{k}'*JpM{k}) + Inertia_Link(k)*(JoL{k}'*JoL{k}) + Inertia_Motor(k)*(JoM{k}'*JoM{k});
end
B = simplify(B);
C = Christoffel(n,B,q,dq);

for h = 1:n
    s = 0;
    for m = 1:n
        s = s-Mass_Link(m)*g0'*JpL{m}(:,h)-Mass_Motor(m)*g0'*JpM{m}(:,h);
    end
    g(h)=s;
end
