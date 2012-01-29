/*************************************************************************
Copyright (c) 1980-2007, Jorge Nocedal.

Contributors:
    * Sergey Bochkanov (ALGLIB project). Translation from FORTRAN to
      pseudocode.

This software is freely available for educational or commercial  purposes.
We expect that all publications describing work using this software  quote
at least one of the references given below:
    * J. Nocedal. Updating  Quasi-Newton  Matrices  with  Limited  Storage
      (1980), Mathematics of Computation 35, pp. 773-782.
    * D.C. Liu and J. Nocedal. On the  Limited  Memory  Method  for  Large
      Scale  Optimization  (1989),  Mathematical  Programming  B,  45,  3,
      pp. 503-528.
*************************************************************************/

#ifndef _lbfgs_h
#define _lbfgs_h

#include "ap.h"

/*-----------------------------------------------
This routines must be defined by you:

void funcgrad(ap::real_1d_array x, double& f, ap::real_1d_array& g);

-----------------------------------------------*/

void funcgrad(ap::real_1d_array x, double& f, ap::real_1d_array& g)
{
	
}

/*************************************************************************
        LIMITED MEMORY BFGS METHOD FOR LARGE SCALE OPTIMIZATION
                          JORGE NOCEDAL

The subroutine minimizes function F(x) of N arguments by  using  a  quasi-
Newton method (LBFGS scheme) which is optimized to use  a  minimum  amount
of memory.

The subroutine generates the approximation of an inverse Hessian matrix by
using information about the last M steps of the algorithm  (instead of N).
It lessens a required amount of memory from a value  of  order  N^2  to  a
value of order 2*N*M.

This subroutine uses the FuncGrad subroutine which calculates the value of
the function F and gradient G in point X. The programmer should define the
FuncGrad subroutine by himself.  It  should  be  noted that the subroutine
doesn't need to waste time for memory allocation of array G,  because  the
memory is allocated in calling the subroutine. Setting a dimension of array
G  each  time  when  calling  a  subroutine  will excessively slow down an
algorithm.

The programmer could also redefine the LBFGSNewIteration subroutine  which
is called on each new step. The current point X, the function value F  and
the  gradient  G  are  passed  into  this  subroutine. It is reasonable to
redefine the subroutine for better debugging, for  example,  to  visualize
the solution process.

Input parameters:
    N   -   problem dimension. N>0
    M   -   number of corrections in the BFGS scheme of Hessian
            approximation update. Recommended value:  3<=M<=7. The smaller
            value causes worse convergence, the bigger will  not  cause  a
            considerably better convergence, but will cause a fall in  the
            performance. M<=N.
    X   -   initial solution approximation.
            Array whose index ranges from 1 to N.
    EpsG -  positive number which  defines  a  precision  of  search.  The
            subroutine finishes its work if the condition ||G|| < EpsG  is
            satisfied, where ||.|| means Euclidian norm, G - gradient, X -
            current approximation.
    EpsF -  positive number which  defines  a  precision  of  search.  The
            subroutine finishes its work if on iteration  number  k+1  the
            condition |F(k+1)-F(k)| <= EpsF*max{|F(k)|, |F(k+1)|, 1}    is
            satisfied.
    EpsX -  positive number which  defines  a  precision  of  search.  The
            subroutine finishes its work if on iteration number k+1    the
            condition |X(k+1)-X(k)| <= EpsX is fulfilled.
    MaxIts- maximum number of iterations. If MaxIts=0, the number of
            iterations is unlimited.

Output parameters:
    X   -   solution approximation. Array whose index ranges from 1 to N.
    Info-   a return code:
                    * -1 wrong parameters were specified,
                    * 0 interrupted by user,
                    * 1 relative function decreasing is less or equal to EpsF,
                    * 2 step is less or equal EpsX,
                    * 4 gradient norm is less or equal to EpsG,
                    * 5 number of iterations exceeds MaxIts.

FuncGrad routine description. User-defined.
Input parameters:
    X   -   array whose index ranges from 1 to N.
Output parameters:
    F   -   function value at X.
    G   -   function gradient.
            Array whose index ranges from 1 to N.
The memory for array G has already been allocated in the calling subroutine,
and it isn't necessary to allocate it in the FuncGrad subroutine.
*************************************************************************/
void lbfgsminimize(const int& n,
     const int& m,
     ap::real_1d_array& x,
     const double& epsg,
     const double& epsf,
     const double& epsx,
     const int& maxits,
     int& info);


#endif
