#pragma once
#include <iostream>
#include <cmath>
#include <vector>

#define PI 3.141592
#define Pos 3
#define Rpy 3
#define ROWS 4
#define COLS 4

class Inverse_Kinematics
{
private:
	std::vector <double> tmp_vector;
	double tmp;

protected:
	/**********************************************
		pass
	***********************************************/
public:
	Inverse_Kinematics(
		std::vector <double>& POS,
		std::vector <double>& RPY);
	void POSRPY_TO_HT(
		std::vector <double> POS,
		std::vector <double> RPY,
		std::vector<std::vector<double>>& Matrix);
	void INV_KIN();
	void SHOW(
		std::vector <double> POS,
		std::vector <double> RPY,
		std::vector<std::vector<double>> Matrix);
};

