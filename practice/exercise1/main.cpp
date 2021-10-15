#include "Homogeneous_transformation.h"
using namespace std;

int main() {
	// variable
	/*
		함수 매개변수로 받기 위해 변수 따로 선언
	*/
	std::vector<std::vector<int>> Matrix1;
	std::vector<std::vector<int>> Matrix2;
	std::vector<std::vector<int>> Result_Matrix;
	std::vector<std::vector<int>> Inverse_Matrix;

	Homogeneous_transformation HT(Matrix1, Matrix2);

	// process
	HT.HT_Inverse(Matrix1, Inverse_Matrix);
	HT.HT_Multiply(
		Inverse_Matrix,
		Matrix2,
		Result_Matrix);
	int Case;
	cout << "행렬 연산 전 과정에 대한 결과를 보기 위해서는 0번 \n최종 R_T_H만 보려면은 1번" << endl;
	cin >> Case;
	switch (Case) {
	case 0:
		HT.Show(Matrix1, Matrix2, Result_Matrix, Inverse_Matrix);
		break;
	case 1:
		HT.Show(Result_Matrix);
		break;
	}
}