function [C] = Christoffel(n,B,q,dq)
for i = 1:n
    for j = 1:n
        for k = 1:n
            c(i,j,k) = 0.5*(diff(B(i,j),q(k))+diff(B(i,k),q(j))-diff(B(j,k),q(i)));
        end
    end
end


for i = 1:n
    for j = 1:n
        s = 0;        
        for k = 1:n
            s = s+c(i,j,k)*dq(k);
        end
        C(i,j) = s;
    end
    
end