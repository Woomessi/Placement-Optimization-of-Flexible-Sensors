clear
clc
syms xs xk x0 ys yk y0 zs zk z0 r n
a = xk^2 + yk^2 + zk^2;
b = 2*(xk*x0 - xs*xk + yk*y0 -ys*yk + zk*z0 - zs*zk);
c = xs^2 + ys^2 + zs^2 + x0^2 + y0^2 + z0^2 - 2*xs*x0 - 2*ys*y0 - 2*zs*z0 - r^2;

n1 = solve(a*n^2 + b*n + c == 0, n);
n2 = simplify(n1);
% n = (-b + sqrt(b^2 - 4*a*c))/(2*a);
% 
% e1 = b^2 - 4*a*c;
% e2 = simplify(e1);
