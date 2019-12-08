function [Rot,Trans] = Forward_Kinematics(name,DH_para,typeOfJoint)
B=eye(4);
for i = 1:size(DH_para,1)
    t(i)=DH_para(i,1);
    alp(i)=DH_para(i,2);
    a(i)=DH_para(i,3);
    d(i)= DH_para(i,4);
end
theta=deg2rad(t);
alpha=deg2rad(alp);
for i=1:length(typeOfJoint)
    C=([cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1]);
    B=B*C;
end  
Rot=B(1:3,1:3)
Trans=B(1:3,4)
for i= 1:length(typeOfJoint)
    if typeOfJoint(i)=='R'
        L(i) = Link('d', d(i),'a', a(i), 'alpha', alpha(i));
        q(i)=theta(i);
    elseif typeOfJoint(i)=='P'
        L(i) = Link('theta', theta(i), 'a', a(i), 'alpha', alpha(i));
        L(i).qlim = [theta(i), d(i)]; 
        q(i)=d(i);
    end
end  
%DH table
R = SerialLink(L);
R.name = name;
%Plot
W=[-2 2 -2 2 -2 2];
plot(R,q,'workspace', W)
teach(R)
end

