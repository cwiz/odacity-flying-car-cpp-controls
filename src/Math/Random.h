// Random Number Utilities Header, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once

// random number routines from Numerical Recipes
double gasdev(int &idum);
double ran1(int &idum);
float LimitValue(float, float, float);

inline float ran1_inRange(float min, float max, int& idum)
{
  return (float)(ran1(idum)*(max-min)+min);
}

inline double ran1_inRange(double min, double max, int& idum)
{
  return (double)(ran1(idum)*(max-min)+min);
}

// limit value by min and max value
inline float LimitValue(float value, float min, float max)
{
	if (value > max) {
		return max;
	}

	if (value < min) {
		return min;
	}

	return value;
}
#pragma once
