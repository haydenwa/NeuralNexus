function [c,J] = cost(x,u,Q,R)

Nt = size(x, 2);
c = zeros(1, Nt);
for t = 1:Nt
    c(t) = x(:,t)'*Q*x(:,t) + u(:,t)'*R*u(:,t);
end
J = cumsum(c);


end