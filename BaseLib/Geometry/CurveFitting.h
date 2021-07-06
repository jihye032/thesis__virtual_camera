
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include <vector>

namespace mg
{

bool fitCurveLLS2Order(std::vector< cml::vector3d> &point_set_2d, double &out_b2, double &out_b1, double &out_b0);

};
