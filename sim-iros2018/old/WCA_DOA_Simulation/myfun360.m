function phi = myfun360(x)
global p
ff = p(1)*x^4 + p(2)*x^3 + p(3)*x^2 + p(4)*x + p(5);
%ff = p(1)*x^2 + p(2)*x^1 + p(1);
penalty = 0;
if x < 1
    penalty = 100;
end
if x > 25
    penalty = 100;
end
phi = ff + penalty;
%ff = p(1)*x^3 + p(2)*x^2 + p(3)*x^1 + p(4);
%ff = p(1)*x(1)^2 + p(2)*x(1)^1 + p(3);
end