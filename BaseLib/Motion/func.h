#pragma once


namespace ml {

	double DiffPosture(const ml::Posture& p1, const ml::Posture& p2);
	double DiffPoseLocalPos(const ml::Posture& p1, const ml::Posture& p2);
	double DiffPoseLocalPos(const ml::Motion& m1, int f1, const ml::Motion& m2, int f2);
	double DiffPoseGlobalPos(const ml::Posture& p1, const ml::Posture& p2);
	Motion& stitch(Motion m1, Motion m2, int warp_width = 20);

	std::pair<cml::vector3d, std::vector<cml::vector3d>> difference(const Posture &p2, const Posture &p1);
	void add_difference(Posture &p, std::pair<cml::vector3d, std::vector<cml::vector3d>> &diff, double ratio);

	std::pair<cml::vector3d, cml::vector3d> difference_root(const Posture &p2, const Posture &p1);
	void add_difference_root_direction_height(Posture &p, std::pair<cml::vector3d, cml::vector3d> &diff, double ratio);

	double scalarTransitionFunc(const double t, const double range);

	void warp( ml::Motion * before_m, ml::Motion * after_m );
	void warp_root_direction_height( ml::Motion * before_m, ml::Motion * after_m, int before_warp_width, int after_warp_width );
};

