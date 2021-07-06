#ifdef ODE_EXT

#pragma once

#include "BaseLib/OdeExt/OdeArticulatedBody.h"
#include "BaseLib/OdeExt/PDController.h"

class OdePdArticulatedBody : public OdeArticulatedBody
{
public:
	void InitializePdControls();
	void SetPdTargetPose(const UrdfPose &taget_pose);
	
	void StepPD(double time);

protected:
	std::vector<PDController> pd_controls_;

};









#endif
