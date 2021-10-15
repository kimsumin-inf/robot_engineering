#include "Inverse_Kinematics.h"
using namespace std;

/********************
variable
*********************/
vector <double> POS;
vector <double> RPY;
vector<vector<double>> Matrix;
vector<vector<double>> R_T_H;
vector<double> joint_angle;
vector<vector<double>> Result_joint_angle;
int Case;
int main() {
	Inverse_Kinematics IK(POS, RPY);
	IK.POSRPY_TO_HT(POS, RPY, Matrix, R_T_H);
	cout << endl <<
		"0:theta1 , theta234, +s3  " << endl <<
		"1:theta1 , theta234, -s3  " << endl <<
		"2:theta1 , theta234 + 180 , +s3  " << endl <<
		"3:theta1 , theta234 + 180 , -s3  " << endl <<
		"4:theta1 + 180 , theta234, +s3  " << endl <<
		"5:theta1 + 180, theta234, -s3  " << endl <<
		"6:theta1 + 180, theta234 + 180, +s3  " << endl <<
		"7:theta1 + 180, theta234 + 180, -s3  " << endl;

	for (int i = 0; i < TOTAL_CASE; i++) {
		IK.INV_KIN(R_T_H, joint_angle, Result_joint_angle, i);
	}
	IK.SHOW(POS, RPY, Matrix, R_T_H, Result_joint_angle);
}