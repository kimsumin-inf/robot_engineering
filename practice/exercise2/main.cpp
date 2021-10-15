#include "Inverse_Kinematics.h"
using namespace std;

/********************
variable
*********************/
std::vector <double> POS;
std::vector <double> RPY;
vector<vector<double>> Matrix;




int main() {
	Inverse_Kinematics IK(POS, RPY);
	IK.POSRPY_TO_HT(POS, RPY, Matrix);
	IK.SHOW(POS, RPY, Matrix);
}