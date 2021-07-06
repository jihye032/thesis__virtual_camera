
#pragma once

#include <vector>

namespace mg
{

class Signal1D : public std::vector<double>
{
public:
	Signal1D() {};

	void MakeSmooth();
	void FindValleyPoints(std::vector<int> &out) const;
	double GetSum(int start, int end) const;
	int GetMax() const;
	int GetMax(int start, int end) const;
	double GetMaxValue() const;
	double GetMaxValue(int start, int end) const;
};

};