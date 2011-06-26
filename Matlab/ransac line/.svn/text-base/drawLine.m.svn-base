function varargout = drawLine(lin, LineColor, varargin)
%DRAWLINE draw the line on the current axis
%   drawline(line) draw the line on the current axis. If line is not
%   clipepd by the axis, function return -1.
%
%   ---------
%
%   author : David Legland 
%   INRA - TPV URPOI - BIA IMASTE
%   created the 31/10/2003.
%

%   HISTORY
%   25/05/2004 : add support for multiple lines (loop)
%   23/05/2005 : add support for arguments

lim = get(gca, 'xlim');
xmin = lim(1);
xmax = lim(2);
lim = get(gca, 'ylim');
ymin = lim(1);
ymax = lim(2);

h = zeros(size(lin, 1), 1);
for i=1:size(lin, 1)
	% intersection with axis : x=xmin
	px1 = intersectLines(lin(i,:), [xmin ymin 0 1]);
	px2 = intersectLines(lin(i,:), [xmax ymin 0 1]);
	py1 = intersectLines(lin(i,:), [xmin ymin 1 0]);
	py2 = intersectLines(lin(i,:), [xmin ymax 1 0]);
	
	% sort points along the x coordinate, and  draw a line between
	% the two in the middle
	points = sortrows([px1 ; px2 ; py1 ; py2], 1);
	if points(2,1)>=xmin && points(2,1)<=xmax
        if isfinite(points(3,1))
            h(i)=line(points(2:3, 1), points(2:3, 2),'color',LineColor);
        else
            h(i)=line(points(1:2, 1), points(1:2, 2),'color',LineColor);
        end
        
        if length(varargin)>0
            set(h(i), varargin{:});
        end
 
	else
        h(i)=-1;
	end
end


if nargout>0
    varargout{1}=h;
end