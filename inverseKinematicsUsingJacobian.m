function [qfin,prof]=inverseKinematicsUsingJacobian(DH_para,q_init,typeOfJoint)
q_init;
for i = 1:size(DH_para,1)
    th(i)=DH_para(i,1);
    alp(i)=DH_para(i,2);
    a(i)=DH_para(i,3);
    d(i)= DH_para(i,4);
end
theta = deg2rad(th)
alpha = deg2rad(alp)
q_init = deg2rad(q_init)
for i= 1:length(typeOfJoint)
    if typeOfJoint(i)=='R'
        L(i) = Link('d', d(i),'a', a(i), 'alpha', alpha(i));
        qt(i)=theta(i);
    elseif typeOfJoint(i)=='P'
        L(i) = Link('theta', theta(i), 'a', a(i), 'alpha', alpha(i));
        L(i).qlim = [theta(i), d(i)];
        qt(i)=d(i);
    end
end 
n = 6; 
numberOfOnes = size(DH_para,1);
indexes = randperm(n);
mask = zeros(1, n);
mask(indexes(1:numberOfOnes)) = 1;
mask=mask';
bot = SerialLink([L]);
T = bot.fkine(q_init);
qfin = bot.ikine(T, qt, mask, 'pinv');
% prof = bot.fkine(qfin);
W=[-2 2 -2 2 -2 2];
plot(bot,qfin,'workspace', W)
teach(bot)
end
