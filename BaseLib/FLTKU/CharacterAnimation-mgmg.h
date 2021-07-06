

#pragma once


#include <string>
#include "BaseLib/FLTKU/AnimationFrameListener.h"
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/Motion/ml.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/GL4U/GL_Renderer.h"

namespace mg
{




class CharacterAnimation : public Animation
{
public:
	CharacterAnimation(GL_Renderer* r);
	CharacterAnimation(GL_Renderer* r, ml::Motion *motion, std::string name="");
	virtual ~CharacterAnimation();

	virtual void motion(ml::Motion *m);
	virtual ml::Motion* motion() { return motion_; }

	virtual cml::matrix44d_c global_transf() const;


	virtual int CountFrames() const;
	virtual void Draw(int frame);
	virtual void Draw(int from, int to, int step=10);

protected:
	virtual void DrawPosturePolygon(int frame);
	virtual void DrawPosturePolygon(ml::Posture const &pose);

protected:

	mg::PmBoneGLListInterface *pm_gl_model_;
	ml::Motion *motion_;
};




class MultiCharacterAni : public Animation
{
public:
	MultiCharacterAni(GL_Renderer* r);
	virtual ~MultiCharacterAni();

	virtual void AddMotion(ml::Motion *motion, std::string name="");
	virtual void RemoveAllMotion();

	ml::Motion* motion(int i) { return motions_[i]; }
	std::vector<ml::Motion *>& motions() { return motions_; }
	virtual cml::matrix44d_c global_transf() const;


	virtual int CountFrames() const;
	virtual void Draw(int frame);

protected:
	virtual void DrawPosturePolygon(int motion_id, int frame);

protected:

	std::vector<ml::Motion *> motions_;
	std::vector<mg::PmBoneGLListInterface*> pm_gl_models_;
	std::vector<std::string> motion_names_;

};


};