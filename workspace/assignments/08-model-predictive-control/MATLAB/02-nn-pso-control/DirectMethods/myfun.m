function [f,gradf] = myfun(x,K)
f = x(1:K)'*x(1:K)*x(K+1)^2+x(K+1);
gradf = zeros(K+1,1);
for i=1:K
   gradf(i)=2*x(i)*x(K+1)^2; 
end
gradf(K+1)=2*x(K+1)*(x(1:K)'*x(1:K))+1;
end