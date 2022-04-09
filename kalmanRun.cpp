#include "kalman.h"

#include <random>
#include <iostream>
#include <chrono>


//First wrapper function
void sensor(Kalman* k, double x, double v, double a, std::default_random_engine generator, std::normal_distribution<double> distribution);
//Control updater
Vec calcU();

int main()
{
	const int dt = 1;
	//Random number generator to simulate sensors being bad
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::normal_distribution<double> distribution (-1.0, 1.0);

	//Matrices for sensor zero
	Mat P0, H0, R0;
	P0 << 1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0;
	
	H0 << 1.0, 0.0, 0.0,
		  0.0, 0.0, 0.0,
		  0.0, 0.0, 0.0;

	R0 << 1.0, 0.0, 0.0,
		  0.0, 1.0, 0.0,
		  0.0, 0.0, 1.0;

	
	//Matrices for sensor one
	Mat P1, H1, R1;
	P1 << 1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0;

	H1 << 0.0, 0.0, 0.0,
		  0.0, 1.0, 0.0,
		  0.0, 0.0, 0.0;

	R1 << 1.0, 0.0, 0.0,
		  0.0, 1.0, 0.0,
		  0.0, 0.0, 1.0;

	//Matrices for sensor two
	Mat P2, H2, R2;
	P2 << 1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0,
	      1.0, 1.0, 1.0;

	H2 << 0.0, 0.0, 0.0,
		  0.0, 0.0, 0.0,
		  0.0, 0.0, 1.0;

	R2 << 1.0, 0.0, 0.0,
		  0.0, 1.0, 0.0,
	  	  0.0, 0.0, 1.0;

	//General values
	Mat Q, F, G;
	Q << 0.1, 0.1, 0.1,
		 0.1, 0.1, 0.1,
		 0.1, 0.1, 0.1;

	F << 1.0, dt * 1.0, dt * dt * 0.5,
		 0, 1.0, dt * 1.0,
		 0, 0, 1.0;

	G << 1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
		 0.0, 0.0, 1.0;

	Vec x0; x0 << 0, 0.0, 0.0;

	double r = 0.0;
	double v = 0.0;
	double a = 1.0;

	Mat Parray[3] = {P0, P1, P2};
	Mat Harray[3] = {H0, H1, H2};
	Mat Rarray[3] = {R0, R1, R2};

	Kalman k = Kalman(x0, Q, F, G, Parray, Harray, Rarray);

	while (true) //Test loop
	{
		k.predict(0, calcU());
		k.predict(1, calcU());
		k.predict(2, calcU());
		sensor(&k, r, v, a, generator, distribution);
		std::cout << "#== Real: ==#\n" << r << '\n';
		std::cout << v << '\n' << a << '\n';
		std::cout << "#== Calculated: ==#\n"<< k.getX() << '\n';
		//Physics update
		r = r + v * dt + a * dt * dt / 2;
		v = v + a * dt;
		a = a;
		//Pause for input
		std::cin.get();
	}
}

//First wrapper function
void sensor(Kalman* k, double r, double v, double a, std::default_random_engine generator, std::normal_distribution<double> distribution)
{
	//Add noise to measurements
	double rn = r + distribution(generator);
	double vn = v + distribution(generator);
	double an = a + distribution(generator);
	Vec z; z << rn, vn, an;
	k->correct(0, z);
	k->correct(1, z);
	k->correct(2, z);
}

//Control updater
Vec calcU()
{
	Vec v; v << 0, 0, 0;
	return v;
}