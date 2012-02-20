data = load("PFHTiming.txt");
plot(data(:,1), data(:,2))
%xlabel("Number of neighbors")
xlabel("Number of neighbors","fontsize",18)
ylabel("time (seconds)","fontsize",18)
title("PFH Benchmark (100k points)","fontsize",18)
print -dpdf pfhbenchmark.pdf
print -dpng pfhbenchmark.png
pause