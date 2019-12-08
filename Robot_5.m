classdef Robot_5
    properties
        n
        link_type
        DH
    end
    methods %(Access = private)    
        function [T] = HTM_i(obj,f_i)
            A = obj.DH_HTM();T = eye(4,4);
            for i =1:f_i
                T = T*A{i};
            end
        end
        function [s1] = Func_J(obj)
            T = HTM_i(obj,3);
            p = T(1:3,4);
            q = obj.Joint_C();
            try
            JJ = [diff(p(1),q(1)) diff(p(1),q(2)) diff(p(1),q(3));
                  diff(p(2),q(1)) diff(p(2),q(2)) diff(p(2),q(3));
                  diff(p(3),q(1)) diff(p(3),q(2)) diff(p(3),q(3))];
            s = simplify(det(JJ));
            s1 = arrayfun(@char, s, 'uniform', 0);       %newadd
            s1=strcat(s1,'=0');
            %s==0
            catch
            JJ = obj.Compute_Jacobian();
            m = min(size(JJ,1),size(JJ,2));
            if det(JJ(1:m,1:m)) == 0
                s1 = 'Is Singular';
            else
                s1 = 'Not Singular';
            end
            end
        end
        function [R,o] = return_R(obj,j)
            A = obj.DH_HTM();
            if j == 0
                R = eye(3);o = [0;0;0];
            else
                A1 = eye(4);
                for i = 1:j
                    A1 = A1*A{i};
                end
                R = A1(1:3,1:3);o = A1(1:3,end);
            end
        end
    end
    methods
        function obj = Robot_5(no_links,DH,linktype)
            obj.n = no_links;
            obj.link_type = linktype;
            obj.DH = DH;
        end
        function [A] = DH_HTM(obj)
            for i = 1:size(obj.DH,1)
                theta=obj.DH(i,1);
                alpha=obj.DH(i,2);
                r=obj.DH(i,3);
                d= obj.DH(i,4);
                A{i} = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta);
                        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta);
                        0 sin(alpha) cos(alpha) d;
                        0 0 0 1];
            end
        end
        function [T] = HTM(obj)
            A = obj.DH_HTM();T = eye(4,4);
            for i = 1:size(A,2)
                T = T*A{i};
            end
        end
        function [J] = Compute_Jacobian(obj)
            A = obj.DH_HTM();
            [~,on] = obj.return_R(obj.n);
            Jv = cell(1,obj.n);Jw = cell(1,obj.n);
            for j = 1:size(A,2)
                [R,o] = obj.return_R(j-1); 
                if strcmp(obj.link_type(j),'P')
                    Jv{j} = R(:,end);
                    Jw{j} = [0;0;0];
                else       
                    Jw{j} = R(:,end);
                    try
                    o_ = sym2cell(on-o);
                    x = cross(R(:,end),[o_{1:3}]);
                    catch
                    o_ = on-o;
                    x = cross(R(:,end),[o_(1:3)]);
                    end
                    Jv{j} = [x(1);x(2);x(3)];
                end
            end
            Jk1 = [ ];Jk2 = [];
            for kk = 1:size(A,2)
                Jk1 = [Jk1 Jv{kk}];
                Jk2 = [Jk2 Jw{kk}];
            end
            J = [Jk1;Jk2];
            %J = [Jv{1} Jv{2} Jv{3};Jw{1} Jw{2} Jw{3}];
        end
        function q = Joint_C(obj)
            str1 = obj.link_type; 
            for i=1:length(str1)
                if str1(i) == 'R'
                    q(i) = obj.DH(i,1);
                else
                    q(i) = obj.DH(i,4);
                end
            end
        end
        function[s1] = Compute_Singularity_1(obj)
            k = [];J = obj.Compute_Jacobian();
            for i = 1:size(J,1)
                if all(J(i,:)==0)
                    k = [k;i];
                end
            end
            J(k,:) = [];
            m = min(size(J,1),size(J,2));
            if m<=2
                try
                s = simplify(det(J(1:m,1:m)));
                s1 = arrayfun(@char, s, 'uniform', 0);       %newadd
                s1 = strcat(s1,'=0');
                %s==0
                catch
                det1 = det(J(1:m,1:m));
                if det1==0
                    s1 = 'Is Singular';
                else
                    s1 = 'Not Singular';
                end
                end
            else
                %'use m>3';
                s1 = obj.Func_J();
        
            end
        end
        function [J] = Compute_AnalyticJacobian(obj)
            J = obj.Compute_Jacobian();k = [];
            for i = 1:size(J,1)
                if all(J(i,:)==0)
                    k = [k;i];
                end
            end
            if size(k,1)>2
                k(end,:)=[];
            end
            J(k,:) = [];
        end
        function [RA] = EA(~,varargin)
            if nargin <2
                pm_error('More inputs needed')
            else
                F = varargin{1};I = varargin{2};    %F = Fixed frame/Current frame; I = 'Angles/Rotation Matrix'
                if isequal(I,'Angles')
                    angles = varargin{3};
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
        end
        function R = workspace(obj)
            typeOfJoint = obj.link_type;
            try
            has(obj.DH(),obj.Joint_C());
                
            catch
            DH_para = obj.DH();
            addpath(genpath('rvctools'));
            for i = 1:size(DH_para,1)
                theta(i)=DH_para(i,1);
                alpha(i)=DH_para(i,2);
                a(i)=DH_para(i,3);
                d(i)= DH_para(i,4);
            end
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
            R.name = class(obj);
            W=[-2 2 -2 2 -2 2];
            plot(R,q,'workspace', W)
            teach(R)
            end
        end
        function ik = inverseKinematics(obj,DH_para) 
            typeOfJoint = obj.link_type;
            for i = 1:size(DH_para,1)
                theta(i)=DH_para(i,1);
                alpha(i)=DH_para(i,2);
                a(i)=DH_para(i,3);
                d(i)= DH_para(i,4);
            end
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
        function [qfin,prof]=inverseKinematicsUsingJacobian(obj,DH_para,q_init)
            typeOfJoint = obj.link_type;
            for i = 1:size(DH_para,1)
                theta(i)=DH_para(i,1);
                alpha(i)=DH_para(i,2);
                a(i)=DH_para(i,3);
                d(i)= DH_para(i,4);
            end
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
            numberOfOnes = size(DH_para,1);
            indexes = randperm(6);
            mask = zeros(1, 6);
            mask(indexes(1:numberOfOnes)) = 1;
            mask=mask';
            bot = SerialLink([L]);
            T = bot.fkine(q_init);
            qfin = bot.ikine(T, qt, mask, 'pinv');
            prof = bot.fkine(qfin);
            % W=[-2 2 -2 2 -2 2];
            % plot(bot,q,'workspace', W)
            % teach(bot)
        end
        function [Rot,Trans] = ForwardKinematics(obj)
            typeOfJoint = obj.link_type;
            B=eye(4);
            DH_para = obj.DH();
            for i = 1:size(DH_para,1)
                theta(i)=DH_para(i,1);
                alpha(i)=DH_para(i,2);
                a(i)=DH_para(i,3);
                d(i)= DH_para(i,4);
            end
            for i=1:length(typeOfJoint)
                C=([cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
                    sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
                    0 sin(alpha(i)) cos(alpha(i)) d(i);
                    0 0 0 1]);
                B=B*C;
            end  
            Rot=B(1:3,1:3);
            Trans=B(1:3,4);
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
            R.name = 'Transformations of DH';
            %Plot
            W=[-2 2 -2 2 -2 2];
            plot(R,q,'workspace', W)
            teach(R)
        end
    end
end