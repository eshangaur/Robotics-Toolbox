function ik = inverseKinematics(typeOfJoint,DH_para) 
for i = 1:size(DH_para,1)
    th(i)=DH_para(i,1);
    alp(i)=DH_para(i,2);
    a(i)=DH_para(i,3);
    d(i)= DH_para(i,4);
end
theta = deg2rad(th)
alpha = deg2rad(alp)
for i= 1:length(typeOfJoint)
    if typeOfJoint(i)=='R'
        L(i) = Link('d', d(i),'a', a(i), 'alpha', alpha(i));
    elseif typeOfJoint(i)=='P'
        L(i) = Link('theta', theta(i), 'a', a(i), 'alpha', alpha(i));
        L(i).qlim = [theta(i), d(i)]; 
    end
end  
%DH table
R = SerialLink(L);
fwd_kine = R.fkine(theta);
%inverse kinematics
ik = R.ikunc(fwd_kine);
end