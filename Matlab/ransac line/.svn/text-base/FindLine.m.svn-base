function [A B C] = FindLine(P1, P2)

x1 = P1(1);
x2 = P2(1);
y1 = P1(2);
y2 = P2(2);

epsilon = 1e-6;

if (x2-x1) > epsilon
	m = (y2-y1)/(x2-x1);
	b = y1 - ((y2-y1)/(x2-x1))*x1;
else
	m = 1e6;
	b = -1e6;
end

C = 1;
B = -1/b;
A = -B*m;
