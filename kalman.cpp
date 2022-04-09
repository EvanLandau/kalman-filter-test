#include "kalman.h"

Vec calcState(Mat F, Vec xOld, Mat G, Vec u) //State extrapolation equation
{
	Vec s = (F * xOld) + (G * u);
	std::cout << "Calc state: \n" << s << std::endl;
	return s;
}

Mat calcUncertainty(Mat F, Mat POld, Mat Q) //Covariance extrapolation equation
{
	Mat U = F * POld * F.transpose() + Q;
	std::cout << "Calc uncertainty: \n" << U << std::endl;
	return U;
}

Mat calcGain(Mat POld, Mat H, Mat R) //Kalman gain equation
{   Mat K = POld * (H.transpose()) * (H * POld * (H.transpose()) + R).inverse();
	std::cout << "Calc gain: \n" << K << std::endl;
	return K;
}

Vec updateState(Vec xOld, Mat K, Vec z, Mat H) //State update equation
{
	Vec s =  xOld + K * (z - H*xOld);
	std::cout << "Update state: \n" << s << std::endl;
	return s;
}

Mat updateUncertainty(Mat K, Mat H, Mat POld, Mat R) //Covariance update equation
{
	Mat U = (Mat::Identity() - K * H) * POld * (Mat::Identity() - K * H).transpose() + K * R * K.transpose();
	std::cout << "Update uncertainty: \n" << U << std::endl;
	return U;
}

Kalman::Kalman(Vec x0_, Mat Q_, Mat F_, Mat G_, Mat *P_, Mat *H_, Mat *R_) //Constructor for Kalman filter
{
	x = x0_;
	Q = Q_;
	F = F_;
	G = G_;
	for (int i = 0; i < sensors; i++)
	{
		P[i] = P_[i];
		POld[i] = P_[i];
		H[i] = H_[i];
		R[i] = R_[i];
	}
}

void Kalman::predict(int sensorID, Vec u) //Predict the future
{
	Vec xhold = this->x;
	this->x = calcState(this->F, this->x, this->G, u);
	this->xOld = xhold;
	Mat phold = this->P[sensorID];
	this->P[sensorID] = calcUncertainty(this->F, this->P[sensorID], this->Q);
	this->POld[sensorID] = phold;
}

void Kalman::correct(int sensorID, Vec z)
{
	//Also known as measurement update
	//This should be run with different parameters for each sensor
	//Then predict should be run

	//Are the parameters sensible?
	if (sensorID < 0 || sensorID >= sensors)
	{
		throw std::invalid_argument("Sensor ID out of range.");
	}

	//Compute the Kalman gain
	Mat K = calcGain(this->P[sensorID], this->H[sensorID], this->R[sensorID]);
	//Update estimate with measurement
	Vec xHold = this->x;
	this->x = updateState(this->x, K, z, this->H[sensorID]);
	this->xOld = xHold;
	//Update estimate uncertainty
	Mat Phold = updateUncertainty(K, this->H[sensorID], this->P[sensorID], this->R[sensorID]);
	this->POld[sensorID] = this->P[sensorID];
	this->P[sensorID] = Phold;
}

Vec Kalman::getX()
{
	return this->x;
}