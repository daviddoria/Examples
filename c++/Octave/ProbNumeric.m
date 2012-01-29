function P = ProbNumeric(D, L, Clutter, Mismatch)

if D <= (L - Mismatch)
	
	P = Clutter * exp(-Clutter * D);
	
elseif D > (L - Mismatch)
	
	%initialize params
	Mass1Int(0, Clutter);
	Mass2Int(0, L, Mismatch);
	mass1 = quad("Mass1Int", 0, L-Mismatch);
	mass2 = quad("Mass2Int", L-Mismatch, inf);
	
	alp = (1 - double(mass1))/double(mass2);
	
	%check
	%mass2fixed = alp * double(mass2);
	%double(mass1) + double(mass2)
	%double(mass1) + double(mass2fixed)
	
	P = alp/(Mismatch * sqrt(2*pi)) * exp(-(D-L)^2 / (2*Mismatch^2));
end
