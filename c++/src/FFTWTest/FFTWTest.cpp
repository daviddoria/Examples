#include <iostream>
#include <fftw3.h>

int main(int argc, char *argv[])
{
  fftw_complex *in, *out;
  fftw_plan p;
  unsigned int N = 10;

  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

  fftw_execute(p); // repeat as needed

  fftw_destroy_plan(p);
  fftw_free(in); fftw_free(out);

  return 0;
}