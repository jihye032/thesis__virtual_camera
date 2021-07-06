




#include "BaseLib/FLTKU/CharacterAnimation.h"
#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/Geometry/AABox.h"
// #include "BaseLib/Geometry/GeometryGL.h"
// #include "BaseLib/GLUU/gluu.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include <algorithm>

namespace mg
{

//////////////////////////////////////////////////////////////////////////
// Class: CharacterAnimation

CharacterAnimation::CharacterAnimation()
{
	motion_ = 0;
	skin_ = 0;
	tmp_skin_ = 0;
	tmp_skin_vao_ = 0;
	tmp_skin_vbogroup_ = 0;
	type_ = "CharacterAnimation";
}

CharacterAnimation::CharacterAnimation(GL_Renderer* r) : Animation(r)
{
	motion_ = 0;
	skin_ = 0;
	tmp_skin_ = 0;
	tmp_skin_vao_ = 0;
	tmp_skin_vbogroup_ = 0;
	type_ = "CharacterAnimation";
}

CharacterAnimation::CharacterAnimation(GL_Renderer* r, std::string n) : Animation(r)
{
	motion_ = 0;
	skin_ = 0;
	tmp_skin_ = 0;
	tmp_skin_vao_ = 0;
	tmp_skin_vbogroup_ = 0;

	name(n);

	type_ = "CharacterAnimation";
}

CharacterAnimation::~CharacterAnimation()
{
	DeleteTmpSkin();
}

void
CharacterAnimation::CreateTmpSkin()
{
	DeleteTmpSkin();

	if ( motion_ == nullptr ) return;
	const ml::Body *b = motion_->body();

	tmp_skin_vbogroup_ = CreateMlBodyVBO(b);
	tmp_skin_vao_ = mg::GL_ResourceManager::singleton()->CreateVAO("", tmp_skin_vbogroup_);
	tmp_skin_ = mg::GL_ResourceManager::singleton()->CreateRenderableObj("", tmp_skin_vao_);
	tmp_skin_->flag_skinning(true);

	for ( int i=0; i<(int)b->num_joint(); i++ )
	{
		tmp_skin_->SetBoneOffsetMatrix(i, b->GetGlobalTransf(i));
	}
}

void
CharacterAnimation::DeleteTmpSkin()
{
	if ( tmp_skin_ )
	{
		mg::GL_ResourceManager::singleton()->DeleteRenderableObj(tmp_skin_);
	}

	if ( tmp_skin_vao_ )
	{
		mg::GL_ResourceManager::singleton()->DeleteVAO(tmp_skin_vao_);
	}

	if ( tmp_skin_vbogroup_ )
	{
		mg::GL_ResourceManager::singleton()->DeleteVBOGroup(tmp_skin_vbogroup_);
	}

	tmp_skin_ = 0;
	tmp_skin_vao_ = 0;
	tmp_skin_vbogroup_ = 0;
}


void
CharacterAnimation::motion(ml::Motion *m, mg::GL_RenderableObj *skin)
{
	if ( m==0 ) return;

	this->motion_ = m;
	fps_ = m->fps();
	final_ani_time_ms_ = (AniTime_ms)( 1000 * m->size() / m->fps() )-1;

	skin_ = skin;
	if ( skin_ == nullptr )
	{
		CreateTmpSkin();
		skin_ = tmp_skin_;
	}


	NotifyAnimationUpdated();
}

cml::matrix44d
CharacterAnimation::global_transf() const
{
	cml::matrix44d t;
	t.identity();
	if ( motion_==0 || motion_->size() == 0 ) t;

	cml::transf qm_t = cml::PlaneProject_transf( motion_->posture(0).GetGlobalTransf(0) );
	cml::quaterniond qm_q = cml::QuaternionMatrix(qm_t);
	cml::vector3d qm_v = cml::TranslationMatrix(qm_t);
	
	t *= cml::MatrixRotationQuaternion({qm_q[0], qm_q[1], qm_q[2], qm_q[3]});
	t *= cml::MatrixTranslation(qm_v[0], qm_v[1], qm_v[2]);


	return t;
}

int
CharacterAnimation::CountTotalFrames() const
{
	if ( motion_ == 0 ) return 0;
	return motion_->size();
}

ml::Posture*
CharacterAnimation::GetCurrentPosture()
{
	if (motion_->size() == 0) return nullptr;

	int pose_id = AniTimeMs_to_FrameId(cur_time_ms_);
	if ( pose_id >= (int)motion_->size() ) pose_id = motion_->size() - 1;
	return &motion_->posture(pose_id);
}


void
CharacterAnimation::DrawPosturePolygon(AniTime_ms t)
{
	if (motion_->size() == 0) return;

	int pose_id = AniTimeMs_to_FrameId(t);
	if ( pose_id >= (int)motion_->size() ) pose_id = motion_->size() - 1;
	ml::Posture const &cur_pose = motion_->posture(pose_id);
	DrawPosturePolygon(cur_pose);
}


void
CharacterAnimation::DrawPosturePolygon(ml::Posture const &cur_pose)
{
	for ( unsigned int i=0; i<cur_pose.body()->num_joint(); i++ )
	{
		int j=cur_pose.body()->parent(i);
		skin_->SetBoneMatrix(i, cur_pose.GetGlobalTransf(i));
	}
	mg::GetDefaultRenderer()->Draw(skin_);


}


void
CharacterAnimation::Draw(AnimationViewer *ani_viewer)
{
	
	if ( motion_ == 0 ) return;
	DrawPosturePolygon(cur_time_ms_);
	//drawPostureLine(frame);

}








};