#ifdef ODE_EXT

#pragma once

#include "Ode/ode.h"

void DrawOdeGeom(dGeomID dgeom_id);
void DrawOdeBody(dBodyID dbody_id, bool colored=true);

#endif
