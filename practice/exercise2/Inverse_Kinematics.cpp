#include "Inverse_Kinematics.h"
using namespace std;

double deg2rad(double degree) {
	return degree * PI / 180;
}

double rad2deg(double radian) {
	return radian * 180 / PI;
}

Inverse_Kinematics::Inverse_Kinematics(
	vector <double>& POS, vector <double>& RPY)
	:tmp(0)
{
	//POS< RPY initialize
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
		RPY[i] = deg2rad(RPY[i]);
	}
}
void Inverse_Kinematics::HT_Multiply(
	vector<vector<double>> Matrix1,
	vector<vector<double>> Matrix2,
	vector<vector<double>>& Result_Matrix) {
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			tmp = 0;
			for (int k = 0; k < TEMP; k++) {
				tmp += (Matrix1[i][k] * Matrix2[k][j]);
			}
			tmp_vector.push_back(tmp);
		}
		Result_Matrix.push_back(tmp_vector);
		tmp_vector.clear();
	}

	tmp_vector.clear();
}

void Inverse_Kinematics::POSRPY_TO_HT(

	vector <double> POS,
	vector <double> RPY,
	vector<vector<double>>& Matrix,
	vector<vector<double>>& R_T_H)
{
	cout << fixed;
	cout.precision(3);
	vector<vector<double>> pos;
	//Initialize
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			tmp_vector.push_back(0);
		}
		Matrix.push_back(tmp_vector);
		tmp_vector.clear();
	}
	tmp_vector.clear();
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			tmp_vector.push_back(0);
		}
		pos.push_back(tmp_vector);
		tmp_vector.clear();
	}
	tmp_vector.clear();
	for (int i = 0; i < 3; i++) {
		pos[i][3] = POS[i];
		pos[i][i] = 1;
	}
	//

	pos[3][3] = 1;
	//RPY setting
	Matrix[0][0] = cos(RPY[0]) * cos(RPY[1]);
	Matrix[0][1] =
		(cos(RPY[0]) * sin(RPY[1]) * sin(RPY[2])) -
		(sin(RPY[0]) * cos(RPY[2]));
	Matrix[0][2] =
		(cos(RPY[0]) * sin(RPY[1]) * cos(RPY[2])) +
		(sin(RPY[0]) * sin(RPY[2]));
	Matrix[1][0] = sin(RPY[0]) * cos(RPY[1]);
	Matrix[1][1] =
		(sin(RPY[0]) * sin(RPY[1]) * sin(RPY[2])) +
		(cos(RPY[0]) * cos(RPY[2]));
	Matrix[1][2] =
		(sin(RPY[0]) * sin(RPY[1]) * cos(RPY[2])) -
		(cos(RPY[0]) * sin(RPY[2]));
	Matrix[2][0] =-(sin(RPY[1]));
	Matrix[2][1] = cos(RPY[1]) * sin(RPY[2]);
	Matrix[2][2] = cos(RPY[1]) * cos(RPY[2]);
	Matrix[3][3] = 1;

	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			if (Matrix[i][j] > -0.00001 && Matrix[i][j] < 0.00001) {
				Matrix[i][j] = 0;
			}
		}
	}

	//R_T_H =

	HT_Multiply(pos, Matrix, R_T_H);
}

void Inverse_Kinematics::INV_KIN(
	vector<vector<double>> R_T_H,
	vector<double>& J_A,
	vector<vector<double>>& Result_joint_angle,
	int Case)
{
	tmp = 0;
	double c3 = 0;
	double s3[2] = {};

	s3[0] = pow(1 - pow(c3, 2), 1 / 2);
	s3[1] = -pow(1 - pow(c3, 2), 1 / 2);
	J_A.clear();
	
	for (int i = 0; i < 6; i++)
		J_A.push_back(0);

	switch (Case) {
		//0:theta1 , theta234, +s3
	case 0:

		//INV_KIN
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[0] = sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[0], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[0] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[0] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;

	}
	/*
	//1:theta1 , theta234, -s3
	case 1:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[1] = -sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[1], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[1] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[1] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//2:theta1 , theta234 + 180 , +s3
	case 2:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]);
		tmp = deg2rad(rad2deg((atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2])) + 180));
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[0] = sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[0], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[0] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[0] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//3:theta1 , theta234 + 180 , -s3
	case 3:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + deg2rad(180);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[1] = -sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[0], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[1] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[1] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//4:theta1 + 180 , theta234, +s3
	case 4:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]) + deg2rad(180);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[0] = sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[0], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[0] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[0] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//5:theta1 + 180, theta234, -s3
	case 5:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]) + deg2rad(180);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[1] = -sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[1], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[1] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[1] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//6:theta1 + 180, theta234 + 180, +s3
	case 6:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]) + deg2rad(180);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + deg2rad(180);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[0] = sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[0], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[0] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[0] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}
	//7:theta1 + 180, theta234 + 180, -s3
	case 7:
	{
		J_A[0] = atan2(R_T_H[1][3], R_T_H[0][3]) + deg2rad(180);
		tmp = atan2(R_T_H[2][2], cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + deg2rad(180);
		c3 = (pow((R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), 2) +
			pow((R_T_H[2][3] - sin(tmp) * a4), 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
		s3[1] = -sqrt(1 - pow(c3, 2));
		J_A[2] = atan2(s3[1], c3);
		J_A[1] = atan2((c3 * a3 + a2) * (R_T_H[2][3] - sin(tmp) * a4) - s3[1] * a3 * (R_T_H[0][3] * cos(J_A[0]) +
			R_T_H[1][3] * sin(J_A[0]) - cos(tmp) * a4), (c3 * a3 + a2) * (R_T_H[0][3] * cos(J_A[0]) + R_T_H[1][3] * sin(J_A[0]) -
				cos(tmp) * a4) + s3[1] * a3 * (R_T_H[2][3] - sin(tmp) * a4));
		J_A[3] = tmp - J_A[1] - J_A[2];
		J_A[4] = atan2(cos(tmp) * (cos(J_A[0]) * R_T_H[0][2] + sin(J_A[0]) * R_T_H[1][2]) + sin(tmp) * R_T_H[2][2], sin(J_A[0]) * R_T_H[0][2] -
			cos(J_A[0]) * R_T_H[1][2]);
		J_A[5] = atan2(-sin(tmp) * (cos(J_A[0]) * R_T_H[0][0] + sin(J_A[0]) * R_T_H[1][0]) + cos(tmp) * R_T_H[2][0],
			-sin(tmp) * (cos(J_A[0]) * R_T_H[0][1] + sin(J_A[0]) * R_T_H[1][1]) + cos(tmp) * R_T_H[2][1]);
		Result_joint_angle.push_back(J_A);
		break;
	}

	*/
	}
}


void Inverse_Kinematics::SHOW(
	vector <double> POS,
	vector <double> RPY,
	vector<vector<double>> Matrix,
	vector<vector<double>> R_T_H,
	vector<vector<double>> Joint_Angle)
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
	cout << "R_T_H\n" << endl;
	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			cout << R_T_H[i][j] << "  ";
		}
		cout << endl << endl;
	}

	cout << endl;

	cout << "Joint_Angle(rad)\n" << endl;
	for (int j = 0; j < TOTAL_CASE; j++) {
		cout << j << ' ';
		for (int i = 0; i < 6; i++) {
			cout << Joint_Angle[j][i] << "  ";
		}
		cout << endl << endl;
	}
	cout << endl;
	cout << endl;
	cout << "Joint_Angle(deg)\n" << endl;
	for (int j = 0; j < TOTAL_CASE; j++) {
		cout << j << ' ';
		for (int i = 0; i < 6; i++) {
			cout << rad2deg(Joint_Angle[j][i]) << "  ";
		}
		cout << endl << endl;
	}

	cout << endl;
	for (int i = 0; i < 100; i++) {
		cout << "=";
	}
}