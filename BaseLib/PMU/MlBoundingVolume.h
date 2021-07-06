
#pragma once
#include "BaseLib/Motion/ml.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include <vector>

namespace mg
{

class MlBoundingVolume
{
public:
	MlBoundingVolume();
	virtual ~MlBoundingVolume();
	void Clear();

	void body(const ml::Body *b);
	const ml::Body* body() const { return body_; }

	void CreateDefaultBoundingVolumes(double radius_scale=1.0);
	
	PrimitiveShape* bounding_volume(int joint_id) const { return bounding_volumes_[joint_id]; }
	void bounding_volume(int joint_id, const PrimitiveShape& s);
	
	/// num_bounding_volumes is same as PM_HUMAN_JOINT_NUM
	int num_bounding_volumes() const { return (int)bounding_volumes_.size(); }
	
	void SetPosture(const ml::Posture &pose);

protected:
	
	virtual mg::PrimitiveShape* CreateBonePrimitive(cml::vector3d bone_dir, double radius);
	virtual mg::PrimitiveShape* CreateBonePrimitive(cml::vector3d p0, cml::vector3d p1, double radius);
	virtual mg::PrimitiveShape* CreateBonePrimitive(std::vector< cml::vector3d> &sampling_points);


protected:
	const ml::Body *body_;
	std::vector<PrimitiveShape*> bounding_volumes_;
};

};
