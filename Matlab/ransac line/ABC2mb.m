function [m b]=ABC2mb(A, B, C)
	m=-A/B;
	b = -C/B;