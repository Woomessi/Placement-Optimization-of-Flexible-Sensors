syms t x1 y1 z1 x2 y2 z2 x y z l_sum l l_desire
eqn1 = norm([x-x1,y-y1,z-z1]) == l_desire - l_sum + l;
eqn2 = x == t*x1 + (1-t)*x2;
eqn3 = y == t*y1 + (1-t)*y2;
eqn4 = z == t*z1 + (1-t)*z2;
eqn5 = l_desire > l_sum - l;
eqn6 = t>0;
eqn7 = t<1;
eqn = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6 eqn7];
t = solve(eqn,t);
