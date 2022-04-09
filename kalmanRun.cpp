#include "kalman.h"

#include <random>
#include <iostream>
#include <chrono>


//First wrapper function
void sensor0(Kalman k, double x, double v, double a, std::default_random_engine generator, std::normal_distribution<double> distribution);
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
		  0.0, 1.0, 0.0,
		  0.0, 0.0, 1.0;

	R0 << 1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0,
		  1.0, 1.0, 1.0;

	/*
	//Matrices for sensor one
	Mat P1 = Mat();
	Mat H1 = Mat();
	Mat R1 = Mat();
	void sensor1()
	{

	}

	//Matrices for sensor two
	Mat P2 = Mat();
	Mat H2 = Mat();
	Mat R2 = Mat();
	void sensor2()
	{

	}
	*/

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

	Vec x0; x0 << 0.0, 0.0, 0.0;

	double x, v, a = 0; v, a = 1.0;

	Mat Parray[1] = {P0};
	Mat Harray[1] = {H0};
	Mat Rarray[1] = {R0};

	Kalman k = Kalman(x0, Q, F, G, Parray, Harray, Rarray);

	while (true) //Test loop
	{
		k.predict(0, calcU());
		sensor0(k, x, v, a, generator, distribution);
		std::cout << "Real: " << x << '\n';
		std::cout << v << '\n' << a << '\n';
		std::cout << "Calculated: "<< k.getX() << '\n';
		//Physics update
		x = x + v * dt + a * dt * dt / 2;
		v = v + a * dt;
		a = a;
		//Pause for input
		std::cin.get();
	}
}

//First wrapper function
void sensor0(Kalman k, double x, double v, double a, std::default_random_engine generator, std::normal_distribution<double> distribution)
{
	//Add noise to measurements
	double xn = x + distribution(generator) * 10;
	double vn = v + distribution(generator) * 10;
	double an = a + distribution(generator) * 10;
	Vec z; z << xn, vn, an;
	k.correct(0, z);
}

//Control updater
Vec calcU()
{
	Vec v; v << 0, 0, 0;
	return v;
}