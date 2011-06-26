//#include <iostream>
using namespace std;

const double DegreesPerWave = 360.0;
const double Pi = 3.1415926535897932384626433832795;

class SineWave
{
	public:
	SineWave(){Amplitude = 2.0; Resolution = 256; Wave = new double[256];};
	SineWave(double Amp, int Res){Amplitude = Amp; Resolution = Res; Wave =
	new double[Res]; };
	~SineWave(){delete Wave;};

	void MakeSinWave(double *SineVector);
	void DumpSinWave();
	double Deg2Rad( double x) {return x * Pi/180.0;};

	private:
	double Amplitude;
	int Resolution;
	double *Wave;
};



void SineWave::DumpSinWave()
{
	for ( int i = 0; i < Resolution; i++)
	{
		cout << Wave[i] << endl;
	}
}




void SineWave::MakeSinWave(double *SineVector)
{
	double cnt = 0.0, step = DegreesPerWave / Resolution;

	for ( int i = 0; i < Resolution; i++, cnt += step)
	{
		Wave[i] = Amplitude * sin(Deg2Rad(cnt));
		SineVector[i]=Wave[i];
	}

}
