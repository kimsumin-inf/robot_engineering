#include "Homogeneous_transformation.h"

using namespace std;

Homogeneous_transformation::Homogeneous_transformation(
	std::vector<std::vector<int>>& Matrix1,
	std::vector<std::vector<int>>& Matrix2)
	:tmp(0), sum(0) {
	cout << "U_T_R 입력 " << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cin >> tmp;
			tmp_vec.push_back(tmp);
		}
		Matrix1.push_back(tmp_vec);
		tmp_vec.clear();
	}
	tmp_vec.clear();
	cout << "U_T_H 입력 " << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cin >> tmp;
			tmp_vec.push_back(tmp);
		}
		Matrix2.push_back(tmp_vec);
		tmp_vec.clear();
	}
}

void Homogeneous_transformation::Show(
	std::vector<std::vector<int>> Matrix1,
	std::vector<std::vector<int>> Matrix2,
	std::vector<std::vector<int>> Result_Matrix,
	std::vector<std::vector<int>> Inverse_Matrix) {
	for (int i = 0; i < 100; i++) {
		cout << "=";
	}

	cout << "\nU_T_R" << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cout << Matrix1[i][j] << ' ' << ' ';
		}
		cout << endl;
	}
	cout << endl;
	cout << "U_T_H" << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cout << Matrix2[i][j] << ' ' << ' ';
		}
		cout << endl;
	}
	cout << endl;
	cout << "Result_Matrix (HT_Multiply)" << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cout << Result_Matrix[i][j] << ' ';
		}
		cout << endl;
		cout << endl;
	}
	cout << "Inverse_Matrix (HT_Inverse)" << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cout << Inverse_Matrix[i][j] << ' ';
		}
		cout << endl;

		cout << endl;
	}
	cout << "Rot" << endl;
	for (int i = 0; i < COLS; i++) {
		cout << "Rot: " << Rot[i] << ' ';
	}
	cout << endl;
	cout << endl;
	for (int i = 0; i < 100; i++) {
		cout << "=";
	}
	tmp_vec.clear();
	cout << endl;
	cout << endl;
}

void Homogeneous_transformation::Show(
	std::vector<std::vector<int>> Matrix
) {
	for (int i = 0; i < 100; i++) {
		cout << "=";
	}

	cout << "\nresult" << endl;
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			cout << Matrix[i][j] << ' ';
		}
		cout << endl;
	}

	for (int i = 0; i < 100; i++) {
		cout << "=";
	}
	cout << endl;
}

void Homogeneous_transformation::HT_Multiply(
	std::vector<std::vector<int>> Matrix1,
	std::vector<std::vector<int>> Matrix2,
	std::vector<std::vector<int>>& Result_Matrix) {
	for (int i = 0; i < COLS; i++) {
		for (int j = 0; j < ROWS; j++) {
			tmp = 0;
			for (int k = 0; k < TEMP; k++) {
				tmp += (Matrix1[i][k] * Matrix2[k][j]);
			}
			tmp_vec.push_back(tmp);
		}
		Result_Matrix.push_back(tmp_vec);
		tmp_vec.clear();
	}

	tmp_vec.clear();
}

void Homogeneous_transformation::swap(int& a, int& b) {
	int temp = a;
	a = b;
	b = temp;
}

void Homogeneous_transformation::HT_Inverse(
	std::vector<std::vector<int>> Matrix,
	std::vector<std::vector<int>>& Inverse_Matrix) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			sum += pow(Matrix[i][j], 2);
		}
		Rot.push_back(float(pow(sum, 0.5f)));

		sum = 0;
	}
	sum = 0;
	for (int i = 0; i < 3; i++) {
		sum += pow(Matrix[i][3], 2);
	}
	Rot.push_back(float(pow(sum, 0.5f)));

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			for (int k = 1; k < 3; k++) {
				if (i == j)
					swap(Matrix[i][k], Matrix[k][j]);
			}
		}
	}

	for (int i = 0; i < 3; i++) {
		Matrix[i][3] = -1 * (Rot[i] * Rot[3]);
	}

	Inverse_Matrix = Matrix;
	/*
	swap(Inverse_Matrix[0][1], Inverse_Matrix[1][0]);
	swap(Inverse_Matrix[0][2], Inverse_Matrix[2][0]);
	swap(Inverse_Matrix[1][2], Inverse_Matrix[2][1]);
	*/
}