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

#include <stdafx.h>
#include "lbfgs.h"

void lbfgslincomb(const int& n,
     const double& da,
     const ap::real_1d_array& dx,
     int sx,
     ap::real_1d_array& dy,
     int sy);
double lbfgsdotproduct(const int& n,
     const ap::real_1d_array& dx,
     int sx,
     const ap::real_1d_array& dy,
     int sy);
void lbfgsmcsrch(const int& n,
     ap::real_1d_array& x,
     double& f,
     ap::real_1d_array& g,
     const ap::real_1d_array& s,
     int sstart,
     double& stp,
     const double& ftol,
     const double& xtol,
     const int& maxfev,
     int& info,
     int& nfev,
     ap::real_1d_array& wa,
     const double& gtol,
     const double& stpmin,
     const double& stpmax);
void lbfgsmcstep(double& stx,
     double& fx,
     double& dx,
     double& sty,
     double& fy,
     double& dy,
     double& stp,
     const double& fp,
     const double& dp,
     bool& brackt,
     const double& stmin,
     const double& stmax,
     int& info);
void lbfgsnewiteration(const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g);

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
     int& info)
{
    ap::real_1d_array w;
    double f;
    double fold;
    double tf;
    double txnorm;
    double v;
    ap::real_1d_array xold;
    ap::real_1d_array tx;
    ap::real_1d_array g;
    ap::real_1d_array diag;
    ap::real_1d_array ta;
    bool finish;
    double gnorm;
    double stp1;
    double ftol;
    double stp;
    double ys;
    double yy;
    double sq;
    double yr;
    double beta;
    double xnorm;
    int iter;
    int nfun;
    int point;
    int ispt;
    int iypt;
    int maxfev;
    int bound;
    int npt;
    int cp;
    int i;
    int nfev;
    int inmc;
    int iycn;
    int iscn;
    double xtol;
    double gtol;
    double stpmin;
    double stpmax;

    w.setbounds(1, n*(2*m+1)+2*m);
    g.setbounds(1, n);
    xold.setbounds(1, n);
    tx.setbounds(1, n);
    diag.setbounds(1, n);
    ta.setbounds(1, n);
    funcgrad(x, f, g);
    fold = f;
    iter = 0;
    info = 0;
    if( n<=0||m<=0||m>n||epsg<0||epsf<0||epsx<0||maxits<0 )
    {
        info = -1;
        return;
    }
    nfun = 1;
    point = 0;
    finish = false;
    for(i = 1; i <= n; i++)
    {
        diag(i) = 1;
    }
    xtol = 100*ap::machineepsilon;
    gtol = 0.9;
    stpmin = pow(double(10), double(-20));
    stpmax = pow(double(10), double(20));
    ispt = n+2*m;
    iypt = ispt+n*m;
    for(i = 1; i <= n; i++)
    {
        w(ispt+i) = -g(i)*diag(i);
    }
    gnorm = sqrt(lbfgsdotproduct(n, g, 1, g, 1));
    if( gnorm<=epsg )
    {
        info = 4;
        return;
    }
    stp1 = 1/gnorm;
    ftol = 0.0001;
    maxfev = 20;
    while(true)
    {
        ap::vmove(&xold(1), &x(1), ap::vlen(1,n));
        iter = iter+1;
        info = 0;
        bound = iter-1;
        if( iter!=1 )
        {
            if( iter>m )
            {
                bound = m;
            }
            ys = lbfgsdotproduct(n, w, iypt+npt+1, w, ispt+npt+1);
            yy = lbfgsdotproduct(n, w, iypt+npt+1, w, iypt+npt+1);
            for(i = 1; i <= n; i++)
            {
                diag(i) = ys/yy;
            }
            cp = point;
            if( point==0 )
            {
                cp = m;
            }
            w(n+cp) = 1/ys;
            for(i = 1; i <= n; i++)
            {
                w(i) = -g(i);
            }
            cp = point;
            for(i = 1; i <= bound; i++)
            {
                cp = cp-1;
                if( cp==-1 )
                {
                    cp = m-1;
                }
                sq = lbfgsdotproduct(n, w, ispt+cp*n+1, w, 1);
                inmc = n+m+cp+1;
                iycn = iypt+cp*n;
                w(inmc) = w(n+cp+1)*sq;
                lbfgslincomb(n, -w(inmc), w, iycn+1, w, 1);
            }
            for(i = 1; i <= n; i++)
            {
                w(i) = diag(i)*w(i);
            }
            for(i = 1; i <= bound; i++)
            {
                yr = lbfgsdotproduct(n, w, iypt+cp*n+1, w, 1);
                beta = w(n+cp+1)*yr;
                inmc = n+m+cp+1;
                beta = w(inmc)-beta;
                iscn = ispt+cp*n;
                lbfgslincomb(n, beta, w, iscn+1, w, 1);
                cp = cp+1;
                if( cp==m )
                {
                    cp = 0;
                }
            }
            for(i = 1; i <= n; i++)
            {
                w(ispt+point*n+i) = w(i);
            }
        }
        nfev = 0;
        stp = 1;
        if( iter==1 )
        {
            stp = stp1;
        }
        for(i = 1; i <= n; i++)
        {
            w(i) = g(i);
        }
        lbfgsmcsrch(n, x, f, g, w, ispt+point*n+1, stp, ftol, xtol, maxfev, info, nfev, diag, gtol, stpmin, stpmax);
        if( info!=1 )
        {
            if( info==0 )
            {
                info = -1;
                return;
            }
        }
        nfun = nfun+nfev;
        npt = point*n;
        for(i = 1; i <= n; i++)
        {
            w(ispt+npt+i) = stp*w(ispt+npt+i);
            w(iypt+npt+i) = g(i)-w(i);
        }
        point = point+1;
        if( point==m )
        {
            point = 0;
        }
        if( iter>maxits&&maxits>0 )
        {
            info = 5;
            return;
        }
        lbfgsnewiteration(x, f, g);
        gnorm = sqrt(lbfgsdotproduct(n, g, 1, g, 1));
        if( gnorm<=epsg )
        {
            info = 4;
            return;
        }
        tf = ap::maxreal(fabs(fold), ap::maxreal(fabs(f), 1.0));
        if( fold-f<=epsf*tf )
        {
            info = 1;
            return;
        }
        ap::vmove(&tx(1), &xold(1), ap::vlen(1,n));
        ap::vsub(&tx(1), &x(1), ap::vlen(1,n));
        xnorm = sqrt(lbfgsdotproduct(n, x, 1, x, 1));
        txnorm = ap::maxreal(xnorm, sqrt(lbfgsdotproduct(n, xold, 1, xold, 1)));
        txnorm = ap::maxreal(txnorm, 1.0);
        v = sqrt(lbfgsdotproduct(n, tx, 1, tx, 1));
        if( v<=epsx )
        {
            info = 2;
            return;
        }
        fold = f;
        ap::vmove(&xold(1), &x(1), ap::vlen(1,n));
    }
}


void lbfgslincomb(const int& n,
     const double& da,
     const ap::real_1d_array& dx,
     int sx,
     ap::real_1d_array& dy,
     int sy)
{
    int fx;
    int fy;

    fx = sx+n-1;
    fy = sy+n-1;
    ap::vadd(&dy(sy), &dx(sx), ap::vlen(sy,fy), da);
}


double lbfgsdotproduct(const int& n,
     const ap::real_1d_array& dx,
     int sx,
     const ap::real_1d_array& dy,
     int sy)
{
    double result;
    double v;
    int fx;
    int fy;

    fx = sx+n-1;
    fy = sy+n-1;
    v = ap::vdotproduct(&dx(sx), &dy(sy), ap::vlen(sx,fx));
    result = v;
    return result;
}


void lbfgsmcsrch(const int& n,
     ap::real_1d_array& x,
     double& f,
     ap::real_1d_array& g,
     const ap::real_1d_array& s,
     int sstart,
     double& stp,
     const double& ftol,
     const double& xtol,
     const int& maxfev,
     int& info,
     int& nfev,
     ap::real_1d_array& wa,
     const double& gtol,
     const double& stpmin,
     const double& stpmax)
{
    int infoc;
    int j;
    bool brackt;
    bool stage1;
    double dg;
    double dgm;
    double dginit;
    double dgtest;
    double dgx;
    double dgxm;
    double dgy;
    double dgym;
    double finit;
    double ftest1;
    double fm;
    double fx;
    double fxm;
    double fy;
    double fym;
    double p5;
    double p66;
    double stx;
    double sty;
    double stmin;
    double stmax;
    double width;
    double width1;
    double xtrapf;
    double zero;
    double mytemp;

    sstart = sstart-1;
    p5 = 0.5;
    p66 = 0.66;
    xtrapf = 4.0;
    zero = 0;
    funcgrad(x, f, g);
    infoc = 1;
    info = 0;
    if( n<=0||stp<=0||ftol<0||gtol<zero||xtol<zero||stpmin<zero||stpmax<stpmin||maxfev<=0 )
    {
        return;
    }
    dginit = 0;
    for(j = 1; j <= n; j++)
    {
        dginit = dginit+g(j)*s(j+sstart);
    }
    if( dginit>=0 )
    {
        return;
    }
    brackt = false;
    stage1 = true;
    nfev = 0;
    finit = f;
    dgtest = ftol*dginit;
    width = stpmax-stpmin;
    width1 = width/p5;
    for(j = 1; j <= n; j++)
    {
        wa(j) = x(j);
    }
    stx = 0;
    fx = finit;
    dgx = dginit;
    sty = 0;
    fy = finit;
    dgy = dginit;
    while(true)
    {
        if( brackt )
        {
            if( stx<sty )
            {
                stmin = stx;
                stmax = sty;
            }
            else
            {
                stmin = sty;
                stmax = stx;
            }
        }
        else
        {
            stmin = stx;
            stmax = stp+xtrapf*(stp-stx);
        }
        if( stp>stpmax )
        {
            stp = stpmax;
        }
        if( stp<stpmin )
        {
            stp = stpmin;
        }
        if( brackt&&(stp<=stmin||stp>=stmax)||nfev>=maxfev-1||infoc==0||brackt&&stmax-stmin<=xtol*stmax )
        {
            stp = stx;
        }
        for(j = 1; j <= n; j++)
        {
            x(j) = wa(j)+stp*s(j+sstart);
        }
        funcgrad(x, f, g);
        info = 0;
        nfev = nfev+1;
        dg = 0;
        for(j = 1; j <= n; j++)
        {
            dg = dg+g(j)*s(j+sstart);
        }
        ftest1 = finit+stp*dgtest;
        if( brackt&&(stp<=stmin||stp>=stmax)||infoc==0 )
        {
            info = 6;
        }
        if( stp==stpmax&&f<=ftest1&&dg<=dgtest )
        {
            info = 5;
        }
        if( stp==stpmin&&(f>ftest1||dg>=dgtest) )
        {
            info = 4;
        }
        if( nfev>=maxfev )
        {
            info = 3;
        }
        if( brackt&&stmax-stmin<=xtol*stmax )
        {
            info = 2;
        }
        if( f<=ftest1&&fabs(dg)<=-gtol*dginit )
        {
            info = 1;
        }
        if( info!=0 )
        {
            return;
        }
        mytemp = ftol;
        if( gtol<ftol )
        {
            mytemp = gtol;
        }
        if( stage1&&f<=ftest1&&dg>=mytemp*dginit )
        {
            stage1 = false;
        }
        if( stage1&&f<=fx&&f>ftest1 )
        {
            fm = f-stp*dgtest;
            fxm = fx-stx*dgtest;
            fym = fy-sty*dgtest;
            dgm = dg-dgtest;
            dgxm = dgx-dgtest;
            dgym = dgy-dgtest;
            lbfgsmcstep(stx, fxm, dgxm, sty, fym, dgym, stp, fm, dgm, brackt, stmin, stmax, infoc);
            fx = fxm+stx*dgtest;
            fy = fym+sty*dgtest;
            dgx = dgxm+dgtest;
            dgy = dgym+dgtest;
        }
        else
        {
            lbfgsmcstep(stx, fx, dgx, sty, fy, dgy, stp, f, dg, brackt, stmin, stmax, infoc);
        }
        if( brackt )
        {
            if( fabs(sty-stx)>=p66*width1 )
            {
                stp = stx+p5*(sty-stx);
            }
            width1 = width;
            width = fabs(sty-stx);
        }
    }
}


void lbfgsmcstep(double& stx,
     double& fx,
     double& dx,
     double& sty,
     double& fy,
     double& dy,
     double& stp,
     const double& fp,
     const double& dp,
     bool& brackt,
     const double& stmin,
     const double& stmax,
     int& info)
{
    bool bound;
    double gamma;
    double p;
    double q;
    double r;
    double s;
    double sgnd;
    double stpc;
    double stpf;
    double stpq;
    double theta;

    info = 0;
    if( brackt&&(stp<=ap::minreal(stx, sty)||stp>=ap::maxreal(stx, sty))||dx*(stp-stx)>=0||stmax<stmin )
    {
        return;
    }
    sgnd = dp*(dx/fabs(dx));
    if( fp>fx )
    {
        info = 1;
        bound = true;
        theta = 3*(fx-fp)/(stp-stx)+dx+dp;
        s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
        gamma = s*sqrt(ap::sqr(theta/s)-dx/s*(dp/s));
        if( stp<stx )
        {
            gamma = -gamma;
        }
        p = gamma-dx+theta;
        q = gamma-dx+gamma+dp;
        r = p/q;
        stpc = stx+r*(stp-stx);
        stpq = stx+dx/((fx-fp)/(stp-stx)+dx)/2*(stp-stx);
        if( fabs(stpc-stx)<fabs(stpq-stx) )
        {
            stpf = stpc;
        }
        else
        {
            stpf = stpc+(stpq-stpc)/2;
        }
        brackt = true;
    }
    else
    {
        if( sgnd<0 )
        {
            info = 2;
            bound = false;
            theta = 3*(fx-fp)/(stp-stx)+dx+dp;
            s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
            gamma = s*sqrt(ap::sqr(theta/s)-dx/s*(dp/s));
            if( stp>stx )
            {
                gamma = -gamma;
            }
            p = gamma-dp+theta;
            q = gamma-dp+gamma+dx;
            r = p/q;
            stpc = stp+r*(stx-stp);
            stpq = stp+dp/(dp-dx)*(stx-stp);
            if( fabs(stpc-stp)>fabs(stpq-stp) )
            {
                stpf = stpc;
            }
            else
            {
                stpf = stpq;
            }
            brackt = true;
        }
        else
        {
            if( fabs(dp)<fabs(dx) )
            {
                info = 3;
                bound = true;
                theta = 3*(fx-fp)/(stp-stx)+dx+dp;
                s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
                gamma = s*sqrt(ap::maxreal(double(0), ap::sqr(theta/s)-dx/s*(dp/s)));
                if( stp>stx )
                {
                    gamma = -gamma;
                }
                p = gamma-dp+theta;
                q = gamma+(dx-dp)+gamma;
                r = p/q;
                if( r<0&&gamma!=0 )
                {
                    stpc = stp+r*(stx-stp);
                }
                else
                {
                    if( stp>stx )
                    {
                        stpc = stmax;
                    }
                    else
                    {
                        stpc = stmin;
                    }
                }
                stpq = stp+dp/(dp-dx)*(stx-stp);
                if( brackt )
                {
                    if( fabs(stp-stpc)<fabs(stp-stpq) )
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                }
                else
                {
                    if( fabs(stp-stpc)>fabs(stp-stpq) )
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                }
            }
            else
            {
                info = 4;
                bound = false;
                if( brackt )
                {
                    theta = 3*(fp-fy)/(sty-stp)+dy+dp;
                    s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dy), fabs(dp)));
                    gamma = s*sqrt(ap::sqr(theta/s)-dy/s*(dp/s));
                    if( stp>sty )
                    {
                        gamma = -gamma;
                    }
                    p = gamma-dp+theta;
                    q = gamma-dp+gamma+dy;
                    r = p/q;
                    stpc = stp+r*(sty-stp);
                    stpf = stpc;
                }
                else
                {
                    if( stp>stx )
                    {
                        stpf = stmax;
                    }
                    else
                    {
                        stpf = stmin;
                    }
                }
            }
        }
    }
    if( fp>fx )
    {
        sty = stp;
        fy = fp;
        dy = dp;
    }
    else
    {
        if( sgnd<0.0 )
        {
            sty = stx;
            fy = fx;
            dy = dx;
        }
        stx = stp;
        fx = fp;
        dx = dp;
    }
    stpf = ap::minreal(stmax, stpf);
    stpf = ap::maxreal(stmin, stpf);
    stp = stpf;
    if( brackt&&bound )
    {
        if( sty>stx )
        {
            stp = ap::minreal(stx+0.66*(sty-stx), stp);
        }
        else
        {
            stp = ap::maxreal(stx+0.66*(sty-stx), stp);
        }
    }
}


void lbfgsnewiteration(const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g)
{

}



