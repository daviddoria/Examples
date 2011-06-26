LIBS = -lstdc++ -L/usr/local/lib -openmp

COMPILER = icc

FLAGS = -fPIC

test: test.cpp
	$(COMPILER) test.cpp -o test $(LIBS) $(FLAGS)
