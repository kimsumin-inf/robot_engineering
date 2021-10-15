#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#define ROWS 4
#define COLS 4
#define TEMP 4

class Homogeneous_transformation
{
private:
	/*
		2���� vector�� ���� �ֱ� ���� tmp_vec
		n, o, a, p�� ũ�� ���� ������ �ִ� Rot ����
	*/
	std::vector<int> tmp_vec;
	std::vector<float> Rot;

	// vector �� ���� �ֱ� ���� ���� tmp
	int tmp;
	float sum;
public:
	Homogeneous_transformation(std::vector<std::vector<int>>& Matrix1,
		std::vector<std::vector<int>>& Matrix2);

	void Show(std::vector<std::vector<int>> Matrix1,
		std::vector<std::vector<int>> Matrix2,
		std::vector<std::vector<int>> Result_Matrix,
		std::vector<std::vector<int>> Inverse_Matrix);

	void Show(
		std::vector<std::vector<int>> show);

	void HT_Multiply(
		std::vector<std::vector<int>> Matrix1,
		std::vector<std::vector<int>> Matrix2,
		std::vector<std::vector<int>>& Result_Matrix);

	void HT_Inverse(
		std::vector<std::vector<int>> Matrix,
		std::vector<std::vector<int>>& Inverse_Matrix);

	void swap(int& a, int& b);
};
