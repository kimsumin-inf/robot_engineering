#pragma once
#include <iostream>
#include <cmath>
#include <vector>

#define PI 3.141592
#define Pos 3
#define Rpy 3
#define ROWS 4
#define COLS 4
#define TEMP 4
#define a2 1
#define a3 1
#define a4 0.1

#define TOTAL_CASE 8
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
		std::vector <double>& RPY
	);
	void HT_Multiply(
		std::vector<std::vector<double>> Matrix1,
		std::vector<std::vector<double>> Matrix2,
		std::vector<std::vector<double>>& Result_Matrix
	);
	void POSRPY_TO_HT(
		std::vector <double> POS,
		std::vector <double> RPY,
		std::vector<std::vector<double>>& Matrix,
		std::vector<std::vector<double>>& R_T_H
	);
	void INV_KIN(
		std::vector<std::vector<double>> R_T_H,
		std::vector<double>& Joint_Angle,
		std::vector<std::vector<double>>& Result_joint_angle,
		int Case
	);
	void SHOW(
		std::vector <double> POS,
		std::vector <double> RPY,
		std::vector<std::vector<double>> Matrix,
		std::vector<std::vector<double>> R_T_H,
		std::vector<std::vector<double>>  Joint_Angle
	);
};
