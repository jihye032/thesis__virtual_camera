

#pragma once


#include <string>
#include "BaseLib/FLTKU/AnimationTimeListener.h"
#include "BaseLib/FLTKU/Animation.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/Motion/ml.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/GL4U/GL_VBOVAO.h"
#include "BaseLib/GL4U/GL_Renderer.h"

namespace mg
{




class CharacterAnimation : public Animation
{
public:
	CharacterAnimation();
	CharacterAnimation(GL_Renderer* r);
	CharacterAnimation(GL_Renderer* r, std::string name="");
	virtual ~CharacterAnimation();

	virtual int CountTotalFrames() const override;
	virtual void motion(ml::Motion *m, mg::GL_RenderableObj *skin=nullptr);
	virtual ml::Motion* motion() { return motion_; }

	virtual cml::matrix44d global_transf() const;


	virtual void Draw(AnimationViewer *ani_viewer) override;

protected:
	virtual ml::Posture* GetCurrentPosture();
	virtual void CreateTmpSkin();
	virtual void DeleteTmpSkin();
	virtual void DrawPosturePolygon(AniTime_ms t);
	virtual void DrawPosturePolygon(ml::Posture const &pose);

protected:

	mg::GL_RenderableObj *skin_;
	mg::GL_RenderableObj *tmp_skin_;
	mg::GL_VAO *tmp_skin_vao_;
	mg::GL_VBOGroup *tmp_skin_vbogroup_;
	ml::Motion *motion_;
};



};