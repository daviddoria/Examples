function d = DistancePointToLine(A, B, C, P)
m=P(1);
n=P(2);
d=abs((A*m + B*n + C)/sqrt(A^2 + B^2));