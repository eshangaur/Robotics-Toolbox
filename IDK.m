function [q_] = IDK(r,f,qin,time)
    syms t;
    n=length(r.link_type);
    Ja = r.Compute_AnalyticJacobian();
  
    f=deg2rad(f);
    q(:,1) = r.Joint_C();
    f_jc(:,1)=f.*t;
    H = r.HTM();
    p = H(1:3,4);
    phi = 0;
    Kr=1*eye(4,4);

    qin=deg2rad(qin);
    
    for i = 1:n
        if r.link_type(i) == 'R'
            phi = phi+q(i);
        end               
    end
    for kk = 1:size(q,1)
        for jj = 1:size(q,1)
            Ja(:,kk) = subs(Ja(:,kk),q(jj),f_jc(jj));
%             Ja(2,kk) = subs(Ja(2,kk),q(jj),f_jc(jj));
%             Ja(3,kk) = subs(Ja(3,kk),q(jj),f_jc(jj));
%             Ja(4,kk) = subs(Ja(4,kk),q(jj),f_jc(jj)); 
        end
    end    
    for kk = 1:size(q,1)
        if has(p(1),q(kk))
            p(1) = subs(p(1),q(kk),f_jc(kk));
        end
        if has(p(2),q(kk))
            p(2) = subs(p(2),q(kk),f_jc(kk));
        end
        if has(p(3),q(kk))
            p(3) = subs(p(3),q(kk),f_jc(kk));
        end
    end
    for kk = 1:size(q,1)
        if has(phi(1),q(kk))
            phi(1) = subs(phi(1),q(kk),f_jc(kk));
        end
    end

%     p = eval(p);
    p_dot = diff(p,t);phi_dot = diff(phi,t);
    t_1 = 0:0.1:time;q(:,1)=qin;
    for i = 1:length(t_1)
        pdot(:,i) = subs(p_dot,t,t_1(i));
        phidot(:,i) = subs(phi_dot,t,t_1(i));
        p1(:,i) = subs(p,t,t_1(i));
        phi1(:,i) = subs(phi,t,t_1(i));
        xe(1:3,i) = p1(:,i);
        xe(4,i)=phi1(:,1);
        Jacob = subs(Ja,t,t_1(i));
        if size(Jacob,1)==size(Jacob,2)
            iJacob=inv(Jacob);
        elseif size(Jacob,1)~=size(Jacob,2) 
            iJacob=pinv(Jacob);
        end
        e(:,i)=[p1(:,i); phi1(i)] - xe(:,i);
        x_dot=[pdot(:,i); phidot(i)];
        qdot = iJacob*(x_dot+Kr*e(:,i));
        qq=q(:,i)+qdot*0.001;
        q(:,i+1)= eval(qq);
    end
    q_=eval(q(:,end));
end