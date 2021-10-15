#include "Inverse_Kinematics.h"
using namespace std;


double deg2rad(double degree) {
	return degree * PI / 180;
}

Inverse_Kinematics::Inverse_Kinematics(
	vector <double>& POS, vector <double>& RPY)
	:tmp(0)
{	
	cout << "POS 입력" << endl;
	for (int i = 0; i < Pos; i++) {
		cin >> tmp;
		POS.push_back(tmp);
	}
	cout << endl;
	cout << "RPY 입력" << endl;
	for (int i = 0; i < Rpy; i++) {
		cin >> tmp;
		RPY.push_back(tmp);
	}
	cout << endl;
	
	for (int i = 0; i < Rpy; i++) {
		
		RPY[i] = cos(deg2rad(RPY[i]));
	}
	
}

void Inverse_Kinematics::POSRPY_TO_HT(
	vector <double> POS,
	vector <double> RPY,
	vector<vector<double>>&Matrix)
{
	//Initialize
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			tmp_vector.push_back(0);
		}
		Matrix.push_back(tmp_vector);
		tmp_vector.clear();
	}
	for (int i = 0; i < 3; i++) {
		Matrix[i][3] = POS[i];
	}
	Matrix[0][0]=cos(RPY[0])*cos(RPY[1]);
	Matrix[0][1]=
		(cos(RPY[0])*sin(RPY[1])*sin(RPY[2]))-
		(sin(RPY[0])*cos(RPY[2]));
	Matrix[0][2]=
		(cos(RPY[0]) * sin(RPY[1]) * sin(RPY[2])) +
		(sin(RPY[0]) * cos(RPY[2]));
	Matrix[1][0]= sin(RPY[0]) * cos(RPY[1]);
	Matrix[1][1]= 
		(sin(RPY[0]) * sin(RPY[1]) * sin(RPY[2])) +
		(cos(RPY[0]) * cos(RPY[2]));
	Matrix[1][2]=
		(sin(RPY[0]) * sin(RPY[1]) * cos(RPY[2])) -
		(cos(RPY[0]) * sin(RPY[2]));
	Matrix[2][0]=
		-sin(RPY[2]);
	Matrix[2][1] = cos(RPY[0]) * cos(RPY[2]);
	Matrix[2][2] = cos(RPY[1]) * cos(RPY[2]);
	Matrix[3][3] = 1;

	

	
}

void Inverse_Kinematics::INV_KIN()
{
}

void Inverse_Kinematics::SHOW(
	std::vector <double> POS,
	std::vector <double> RPY,
	std::vector<std::vector<double>> Matrix)
{

	for (int i = 0; i < 100; i++) {
		cout << "=";
	}
	cout << endl;

	cout << "POS\n" << endl;
	for (int i = 0; i < POS.size(); i++) {
		cout << POS[i] << ' ';
	}
	cout << endl;

	cout << "RPY\n" << endl;
	for (int i = 0; i < RPY.size(); i++) {
		cout << RPY[i] << ' ';
	}

	cout << endl;

	cout << "Matrix\n" << endl;
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			cout << Matrix[i][j] << "  ";
		}
		cout << endl << endl;
	}

	cout << endl;
	for (int i = 0; i < 100; i++) {
		cout << "=";
	}

}
