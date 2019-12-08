function R = workspace(name,typeOfJoint,DH_para) 
addpath(genpath('rvctools'));
DH_para
for i = 1:size(DH_para,1)
    t(i)=DH_para(i,1);
    alp(i)=DH_para(i,2);
    a(i)=DH_para(i,3);
    d(i)= DH_para(i,4);
end
theta=deg2rad(t);
alpha=deg2rad(alp);
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
%workspace
R.name = name;
W=[-2 2 -2 2 -2 2];
plot(R,q,'workspace', W)
teach(R)
end
