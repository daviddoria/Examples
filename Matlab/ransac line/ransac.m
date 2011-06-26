%% This is an example usage of the RANSAC algorithm. We try to find the
%% best line though a set of data points which have many outliers.

%setup points
clear; close all; clc;

square_size = 10;
%vectarrow([0 0], [square_size square_size]);

figure;
a = axes;

vectarrow([1 2], [-5 7])
hold(a);

% select the points
disp('Left mouse button picks points.')
disp('Right mouse button picks last point.')
but = 1;
n = 0;
x = []; y = [];

while length(x) < 10
    [xi,yi,but] = ginput(1);
	disp(['Selected point ' num2str(length(x) + 1) ' : (' num2str(xi) ', ' num2str(yi) ')']);

    plot(a,xi,yi,'ro')
    n = n + 1;
    x(n,1) = xi;
    y(n,1) = yi;
end

num_points = 2*length(y);

%make half of the points random
x = [x;rand(num_points/2,1)*square_size]
y = [y;rand(num_points/2,1)*square_size]
points = [x y];
plot(x,y,'o')

%% RANSAC
close all;
plot(x,y,'o')
hold on;
InlierThresh = .5;

Trials = 100;
MaxInliers = 0; %initialization value
WinningLineParams = [0 0 0];%initialization value

for iter = 1:Trials
	CurrentInliers = [];
	inliers = 0;
	rand_num = floor(rand(1,2) * num_points); %randomly choose one of the points
	rand_num( rand_num == 0 ) = 1; %there is no point 0
    
    %calculate the line between the two points
	P1 = points(rand_num(1),:);
	P2 = points(rand_num(2),:);
	[A B C] = FindLine(P1, P2);

    %find the distance from every point to the line
	for counter = 1:num_points
		d = DistancePointToLine(A,B,C,points(counter,:));
        %label the point an inlier if its distance to the line is below the threshold
		if d < InlierThresh
			inliers = inliers + 1;
			CurrentInliers(inliers) = counter; %keep track of which points are inliers on wrt this line
		end
	end
	
	%if this is the best line so far, update the winning line
	if inliers > MaxInliers
		WinningLineParams = [A B C];
		MaxInliers = inliers;
		GoodInliers = CurrentInliers;
		GoodPoints = rand_num;
	end
	
end

A = WinningLineParams(1);
B = WinningLineParams(2);
C = WinningLineParams(3);
[m b] = ABC2mb(A, B, C);
dy = -A;
dx = B;

if WinningLineParams == [0 0 0]
	disp('No Line Found')
else
	l = createLine(0,b,dx,dy);
	drawLine(l,'r')
	plot(x(GoodInliers), y(GoodInliers), 'ro')
	plot(x(GoodPoints), y(GoodPoints), 'go')
end