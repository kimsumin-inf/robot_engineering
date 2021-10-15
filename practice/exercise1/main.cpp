#include "Homogeneous_transformation.h"
using namespace std;

int main() {
	// variable
	/*
		�Լ� �Ű������� �ޱ� ���� ���� ���� ����
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
	cout << "��� ���� �� ������ ���� ����� ���� ���ؼ��� 0�� \n���� R_T_H�� �������� 1��" << endl;
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