#include "BisectorSurfaceTestAni5.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "FL/fl.h"
#include <iostream>
#include <queue>



BisectorSurfaceTestAni5::BisectorSurfaceTestAni5()
{
	flag_subview_ = true;
	flag_draw_bisector_ = true;

	camera1_.lookAt({ 300., 300., 300. }, { 0., 0., 0. }, { 0., 1., 0. });
	camera2_.lookAt({ 300., 300., 300. }, { 0., 0., 0. }, { 0., 1., 0. });
	final_ani_time_ms(2000);



	scene_root_ = std::make_unique<mg::GL_SceneRoot>();
	scene_skydome_ = std::make_unique<mg::GL_SceneRoot>();
	scene_playground_ = std::make_unique<mg::GL_SceneRoot>();
	scene_finger_ = std::make_unique<mg::GL_SceneRoot>();
	scene_object_ = std::make_unique<mg::GL_SceneRoot>();

	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_playground_.get(), "../Data/Playground/Playground.dae", "playground");
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_skydome_.get(), "../Data/skydome/skydome.dae", "sky");
	//mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_object_.get(), "../Data/obj/object4_small.obj", "object");
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_object_.get(), "../Data/obj/object3.obj", "object");
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_finger_.get(), "../Data/obj/finger2.obj", "finger");


	scene_playground_->transf().Translate(-5, 0.01, 5);
	scene_playground_->transf().Scale(100, 100, 100);
	scene_skydome_->transf().Scale(100, 100, 100);

	scene_root_->AddCloneOfNodeHierarchy(scene_skydome_.get());
	scene_root_->AddCloneOfNodeHierarchy(scene_playground_.get());
	//scene_root_->AddCloneOfNodeHierarchy(scene_object_.get());
	//scene_root_->AddCloneOfNodeHierarchy(scene_finger_.get());

	//mesh_ = mg::GL_ResourceManager::singleton()->CreateMesh("pg_single_mesh");


	scene_object_->BuildSinglePolygonMesh(&mesh_B_);
	scene_finger_->BuildSinglePolygonMesh(&mesh_joint_obj_);
	//scene_playground_->BuildSinglePolygonMesh(mesh_);

	mesh_B_.ScaleUniformlyVertices(38);
	mesh_B_.TranslateVertices(cml::vector3d(510, 186, -254));
	mesh_B_.Triangulate();
	mesh_joint_obj_.ScaleUniformlyVertices(40);
	mesh_joint_obj_.Triangulate();

	// pointer
	//
	finger_joint[0].zero();
	finger_joint[1].zero();
	finger_joint[2].zero();
	finger_joint[3].zero();
	finger_last_.zero();
	finger_pos_.zero();


	// camera position
	//
	cmr_pos_.zero();
	cmr_pos2_.zero();
	cmr_pos_last_.zero();
	cmr_pos2_last_.zero();
	cnt_pos_.zero();


	// leap motion 손의 개수
	//
	// leap motion을 현재는 사용하지 않기 때문에 0
	num_leap_hands_ = 0;
	contact_obj_ = false;
	turn_num = false;

}

void
BisectorSurfaceTestAni5::SetCurrentTime(mg::AniTime_ms t)
{
	if (t<0) cur_time_ms_ = 0;
	//else if (t >= final_ani_time_ms_)	cur_time_ms_ = final_ani_time_ms_;
	else cur_time_ms_ = t;

	//mg::AniTime_ms pre_time_ms = cur_time_ms_;
	

	static int frmae_id = -1;
	if (frmae_id == GetCurFrameId()) return;
	frmae_id = GetCurFrameId();




	//std::cout << contact_obj_ << std::endl;

	// 카메라 위치에 따라 손가락 위치 수정
	mesh_joint_obj_T_.Assign(mesh_joint_obj_);
	{
		cml::vector3d tmp_f1, tmp_f2;
		tmp_f1 = finger_joint[0];
		tmp_f2 = finger_pos_;
		/*tmp_f1[0] *= -1;
		tmp_f1[2] *= -1;
		tmp_f2[0] *= -1;
		tmp_f2[2] *= -1;*/

		//cml::matrix44d R;
		//cml::matrix_rotation_vec_to_vec(R, -1*cml::z_axis_3D(), cml::normalize(tmp_f2 - tmp_f1));
		//cml::matrix_rotation_vec_to_vec(R, -1*cml::z_axis_3D(), cml::deg2rad(-30.));
		//R.identity();
		cml::matrix44d T = cml::MatrixTranslation(finger_pos_);
		cml::matrix44d T2 = cml::MatrixTranslation(0., -150., -400.);
		cml::matrix44d C_inv = cml::inverse(renderer_->default_camera()->GetGLViewMatrix());
		mesh_joint_obj_T_.TransformVertices(C_inv*T2*T*cml::MatrixRotationEuler(-cml::pi()/2.+ cml::deg2rad(-25), cml::pi(), 0)*cml::MatrixUniformScaling(0.7));
	}

	{
		cml::matrix44d fT = cml::MatrixTranslation(finger_pos_);
		cml::matrix44d T2 = cml::MatrixTranslation(0., -150., -400.);
		cml::matrix44d cT = cml::inverse(renderer_->default_camera()->GetGLViewMatrix());
		pos_ = cml::matrix_get_translation(cT*T2*fT * cml::MatrixRotationEuler(-cml::pi()/2+ cml::deg2rad(-25), cml::pi(), 0)*cml::MatrixUniformScaling(0.7));
	}
	//std::cout << finger_pos_ << std::endl;

	int tmp = std::rand() % mesh_B_.num_vertices();
	double lng = cml::length(pos_ - mesh_B_.vertex(tmp));
	if (lng <= 100)
		contact_obj_ = true;
	else
		contact_obj_ = false;
	//std::cout << contact_obj_ << std::endl;

	if (num_leap_hands_ >= 1 && flag_subview_)
	{
		// set user camera
		//
		if (contact_obj_ == true)
		{
			cmr_pos_ = FindSubCameraPosition();
			cml::vector3d eye(cmr_pos_);
			cml::vector3d center(cnt_pos_);
			cml::vector3d up(cml::vector3d(0, 1, 0));
			camera1_.lookAt(eye, center, up);
		
		

			// set user camera2
			//
			if (0) {
				cmr_pos2_ = FindFignerSubCameraPosition();
				cml::vector3d eye2(cmr_pos2_);
				cml::vector3d center2(pos_);
				cml::vector3d up2(cml::vector3d(0, 1, 0));
				camera2_.lookAt(eye2, center2, up2);
			}

			b_surface_.SetVertices(mesh_B_.num_vertices(), mesh_B_.vertices(),
				mesh_joint_obj_T_.num_vertices(), mesh_joint_obj_T_.vertices());
			b_surface_.Update();
		}


	}
	cmr_pos_last_ = cmr_pos_;
	finger_last_ = pos_;

	NotifyAnimationUpdated();
}


void
BisectorSurfaceTestAni5::Draw(mg::AnimationViewer *w)
{
	int win_w = w->w();
	int win_h = w->h();

	//renderer_->SetModelIdentity();
	renderer_->UseDefaultCamera();
	DrawInSubViewport(0, 0, win_w, win_h, 0);
	cmr_ = renderer_->camera();


	// Right-Top PIP
	if (flag_subview_)
	{
		renderer_->UseCamera(&camera1_);
		DrawInSubViewport2(win_w - win_w / 2, win_h - win_h / 2, win_w / 2, win_h / 2, 3);
	}

	// Left_botton 
	//renderer_->UseCamera(&camera2_);
	//DrawInSubViewport3(0, 0, win_w / 3, win_h / 3, 1);


	renderer_->UseDefaultCamera();

}

void
BisectorSurfaceTestAni5::DrawInSubViewport(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	// Define the subviewport
	glViewport(vp_x, vp_y, vp_w, vp_h);
	glEnable(GL_SCISSOR_TEST);

	// Border
	if (border_width > 0)
	{
		glScissor(vp_x, vp_y, vp_w, vp_h);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Clear
	glScissor(vp_x + border_width, vp_y + border_width, vp_w - 2 * border_width, vp_h - 2 * border_width);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	//renderer_->SetModelIdentity();
	renderer_->Draw(scene_root_.get());
	//renderer_->Draw(mesh_);

	//renderer_->SetModelIdentity();
	renderer_->SetColor(0);
	renderer_->SetColor(1, 0, 0);
	renderer_->DrawMesh(&mesh_B_);


	//renderer_->SetModelIdentity();
	renderer_->SetColor(7);
	renderer_->DrawMesh(&mesh_joint_obj_T_);

	
		
	if (flag_draw_bisector_)
	{
		if (contact_obj_)
		{
			// Right Up - Green
			renderer_->SetColor(0, 1, 0);
			renderer_->DrawSphere(8, cmr_pos_);

			// Left Down - Blue
			//renderer_->SetColor(0, 0, 1);
			//renderer_->DrawSphere(8, cmr_pos2_);

			// Closed Position - Red
			renderer_->SetColor(1, 0, 0);
			renderer_->DrawSphere(8, cnt_pos_);
		}

		// Finger Pointer Position - Black
		//renderer_->SetColor(0, 0, 0);
		//renderer_->DrawSphere(15, pos_);


		renderer_->EnableTransparency(true);
		renderer_->SetColor(3);
		DrawBisector();
		renderer_->EnableTransparency(false);
	}
	


	glDisable(GL_SCISSOR_TEST);
}


void
BisectorSurfaceTestAni5::DrawInSubViewport2(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
	camera1_.setAspectRatio((double)vp_w / vp_h);


	// Define the subviewport
	glViewport(vp_x, vp_y, vp_w, vp_h);
	glEnable(GL_SCISSOR_TEST);

	// Border
	if (border_width > 0)
	{
		glScissor(vp_x, vp_y, vp_w, vp_h);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Clear
	glScissor(vp_x + border_width, vp_y + border_width, vp_w - 2 * border_width, vp_h - 2 * border_width);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderer_->Draw(scene_root_.get());


	renderer_->SetColor(0);
	renderer_->SetColor(1, 0, 0);
	renderer_->DrawMesh(&mesh_B_);

	renderer_->SetColor(7);
	renderer_->DrawMesh(&mesh_joint_obj_T_);

	glDisable(GL_SCISSOR_TEST);
}

void
BisectorSurfaceTestAni5::DrawInSubViewport3(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
	camera2_.setAspectRatio((double)vp_w / vp_h);


	// Define the subviewport
	glViewport(vp_x, vp_y, vp_w, vp_h);
	glEnable(GL_SCISSOR_TEST);

	// Border
	if (border_width > 0)
	{
		glScissor(vp_x, vp_y, vp_w, vp_h);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Clear
	glScissor(vp_x + border_width, vp_y + border_width, vp_w - 2 * border_width, vp_h - 2 * border_width);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderer_->Draw(scene_root_.get());

	renderer_->SetColor(0);
	renderer_->SetColor(1, 0, 0);
	renderer_->DrawMesh(&mesh_B_);

	renderer_->SetColor(7);
	renderer_->DrawMesh(&mesh_joint_obj_T_);

	glDisable(GL_SCISSOR_TEST);
}


static bool CheckBoundOut(cml::vector3d v)
{
	double max_bound = 1000.;
	double min_bound = -1000.;

	for (int i = 0; i < 3; i++)
		if (v[i] > max_bound) return true;

	for (int i = 0; i < 3; i++)
		if (v[i] < min_bound) return true;

	return false;
}

void
BisectorSurfaceTestAni5::DrawBisector()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	renderer_->SetColor(1, 0.1);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for (int i = 0; i<(int)b_faces.size(); i++)
	{
		// Boundary Check
		bool out_bound = false;
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
			{
				out_bound = true;
			}
		}
		if (out_bound) continue;

		renderer_->glBegin(GL_TRIANGLE_FAN);
		renderer_->glNormal(b_normals[i]);
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glEnd();


		renderer_->glBegin(GL_TRIANGLE_FAN);
		renderer_->glNormal(-1 * b_normals[i]);
		for (int fv_i = (int)b_faces[i].size() - 1; fv_i >= 0; fv_i--)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glEnd();
	}




	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//renderer_->DisableAllLight();
	glLineWidth(2);
	renderer_->SetColor(0, 0, 0, 0.5);
	for (int i = 0; i<(int)b_faces.size(); i++)
	{
		// Boundary Check
		bool out_bound = false;
		for (int fv_i = 0; fv_i<(int)b_faces[i].size(); fv_i++)
		{
			if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
			{
				out_bound = true;
			}
		}
		if (out_bound) continue;

		renderer_->glBegin(GL_LINE_STRIP);
		renderer_->glNormal(b_normals[i]);
		for (int fv_i = 0; fv_i < (int)b_faces[i].size(); fv_i++)
		{
			renderer_->glVertex(b_vertices[b_faces[i][fv_i]]);
		}
		renderer_->glVertex(b_vertices[b_faces[i][0]]);
		renderer_->glEnd();


	}
	renderer_->EnableLight(0);




	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


}


static double DotProduct(cml::vector3d a, cml::vector3d b)
{
	if (a.size() != b.size()) // error check
	{
		puts("Error a's size not equal to b's size");
		return -1;  // not defined
	}

	// compute
	double product = 0;

	for (int i = 0; i <= a.size() - 1; i++)
		product += (a[i]) * (b[i]); // += means add to product

	return product;
}

static bool CheckBoundOut_BS_vtx(cml::vector3d v)
{
	double max_bound = 300.;
	double min_bound = -300.;

	for (int i = 0; i < 3; i++)
		if (v[i] > max_bound) return true;

	for (int i = 0; i < 3; i++)
		if (v[i] < min_bound) return true;

	return false;
}


static long long milliseconds_now() {
	static LARGE_INTEGER s_frequency;
	static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
	if (s_use_qpc) {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		return (1000LL * now.QuadPart) / s_frequency.QuadPart;
	}
	else {
		return GetTickCount();
	}
}

static int g_count_for_e_time = 0;
static long long g_acc_elapsed_time = 0;

cml::vector3d
BisectorSurfaceTestAni5::FindSubCameraPosition()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	// BS 가 계산되지 않았을 때는 카메라의 위치를 구하지 않음
	//
	if (!b_sites.size())
	{
		return finger_pos_;
		return cml::vector3d();
	}

	/*mg::Mesh mesh_joint_ = mesh_joint_obj_;
	mesh_joint_.TranslateVertices(finger_pos_);*/


	/////////////////// Time measuring ///////////////////////////////////////////////
	long long start_es_ms = milliseconds_now();
	/////////////////// Time measuring ///////////////////////////////////////////////


	int cmr_pos_id_ = 0;
	double min_dis = 999999999999;
	cml::vector3d env_vtx_, ptr_vtx_;
	env_vtx_.zero();
	ptr_vtx_.zero();

	//std::cout << "site : " << b_sites.size() << std::endl;
	// >> 1.
	// 두 오브젝트 사이의 거리가 가장 짧은 face를 찾음
	for (int i = 0; i < b_sites.size(); i++)
	{
		cml::vector3d a = mesh_B_.vertex(b_sites[i].first);
		cml::vector3d b = mesh_joint_obj_T_.vertex(b_sites[i].second - mesh_B_.num_vertices());
		double dis = cml::length(a - b);
		//std::cout << dis << " ";

		if (min_dis >= dis)
		{
			min_dis = dis;
			env_vtx_ = a;
			ptr_vtx_ = b;

			//std::cout << min_dis << " ";
			// 두 오브젝트 사이의 거리가 가장 짧은 face의 index
			cmr_pos_id_ = i;
		}
	}

	//std::cout << std::endl;

	// >> 2.
	// 1에서 찾은 face의 중심점을 찾음
	cml::vector3d center_of_face(0, 0, 0);
	for (unsigned int i = 0; i < b_faces[cmr_pos_id_].size(); i++)
	{
		center_of_face += b_vertices[b_faces[cmr_pos_id_][i]];

	}
	center_of_face /= (double)b_faces[cmr_pos_id_].size();


	// >> 3.
	// 찾은 face의 중심점은 카메라가 바라볼 center가 된다.
	cnt_pos_ = center_of_face;


	// >> 4.
	// 카메라의 위치를 정해주자.
	//
	// 카메라의 위치 후보들을 구하자
	int *pre_cmr_tmp = new int[b_vertices.size()];
	int pre_num_ = -1;

	for (unsigned int i = 0; i < b_vertices.size(); i++)
	{
		// 처음 최대값으로 초기화
		pre_cmr_tmp[i] = b_vertices.size();
	}

	// 물체 사이의 거리를 카메라 시야를 확보할 원기둥의 지름으로 이용
	int between_length_ = cml::length(env_vtx_ - ptr_vtx_);
	between_length_ = (int)(between_length_)/4;
	//std::cout << "between_length_ : " << between_length_ << std::endl;

	std::set<int> dup_test;
	for (unsigned int f = 0; f < b_faces.size(); f++)
	{
		for (unsigned int i = 0; i < b_faces[f].size(); i++)
		{
			// 같은 vertex는 계산하지 않음
			if (dup_test.count(b_faces[f][i]) > 0)
			{
				continue;
			}
			dup_test.insert(b_faces[f][i]);

			if (cml::length(cnt_pos_ - b_vertices[b_faces[f][i]]) > 200) continue;

			double margin_r = min( 10.0, (0.25*cml::length(cnt_pos_ - b_vertices[b_faces[f][i]])) );
			cml::vector3d margin_v = margin_r * cml::normalize( cnt_pos_ - b_vertices[b_faces[f][i]] );
			mg::LineSegment line_(cnt_pos_- margin_v, b_vertices[b_faces[f][i]]+ margin_v);

			// 카메라 경로 사이에 물체에 겹치는지 확인
			bool contact = false;
			for (unsigned int j = 0; j < mesh_B_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_B_.vertex(j));

				if (dis <= margin_r)
				{
					contact = true;
					break;
				}
			}

			if ( !contact )
			for (unsigned int j = 0; j < mesh_joint_obj_T_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_joint_obj_.vertex(j));

				if (dis <= margin_r)
				{
					contact = true;
					break;
				}
			}

			if (contact == false)
			{
				pre_num_++;
				pre_cmr_tmp[pre_num_] = b_faces[f][i];
			}
		}
	}

	// 찾을 수 있는 camera의 위치가 없으면 이전에 사용했던 카메라의 위치를 이용하고 함수 종료
	if (pre_num_ <= 0) {
		cmr_pos_ = cmr_pos_last_;
		return cmr_pos_last_;
	}


	// 임의로 생성했던 배열을 지움
	int *pre_cmr_ = new int[pre_num_];
	for (int i = 0; i < pre_num_; i++)
	{
		pre_cmr_[i] = pre_cmr_tmp[i];
	}
	delete[] pre_cmr_tmp;

	

	// 5.
	//
	// 보고있는 방향에서 수직인 방향에 위치한 점을 찾음
	cml::vector3d cmr_eye = cmr_->getViewDirection();
	cml::vector3d cmr_pos_now_;
	cmr_pos_now_.zero();

	double dot_min = 10000;
	int dot_num = 0;
	for (int i = 0; i < pre_num_; i++)
	{
		cml::vector3d bs_vtx_ = b_vertices[pre_cmr_[i]];
		bool check_in = CheckBoundOut_BS_vtx(bs_vtx_);

		if (check_in == true)
			continue;

		/*double dot = sqrt(DotProduct(cml::normalize(cmr_eye), cml::normalize(bs_vtx_)));
		double dist = sqrt(cml::length(cmr_pos_last_ - bs_vtx_));
		double dist2 = (cml::length(finger_pos_ - bs_vtx_));

		if (dist2 > 600) continue;  

		double dd = dot + 0.01*dist;

		if (dist >= between_length_ * 3)
			continue;

		if (dot_min > dd)
		{
			dot = dd;
			dot_num = i;
		}
		else
			continue;
*/
		double dot = pow(DotProduct(cml::normalize(cmr_eye), cml::normalize(cnt_pos_ - bs_vtx_)), 2.);
		double dist = cml::length_squared(cmr_pos_last_ - bs_vtx_);
		double best_dist = pow(50 - sqrt(cml::length(cnt_pos_ - bs_vtx_)), 2.0); // 카메라와 타겟 사이 거리는 50 이 적당함.
		double dist2 = (cml::length(finger_pos_ - bs_vtx_));

		//if (dist2 > 600) continue;	// 너무 멀리가면 skip
		//if (cml::length(cnt_pos_ - bs_vtx_) < 50) continue; // 너무 가까워도 skip

		double dd = dot + 0.01*best_dist;// +0.001*dist;

		/*if (dist >= between_length_ * 3)
			continue;*/

		if (dot_min > dd)
		{
			dot = dd;
			dot_num = i;
		}
		else
			continue;

	}


	if (cml::length(cmr_pos_last_ - b_vertices[pre_cmr_[dot_num]]) > 500 
		|| cml::length(cnt_pos_ - b_vertices[pre_cmr_[dot_num]]) > 500 )
	{

		cmr_pos_ = b_vertices[pre_cmr_[dot_num]];
	}
	else
	{
		cmr_pos_ = 0.9* cmr_pos_last_ + 0.1*b_vertices[pre_cmr_[dot_num]]; // 점차적으로 움직이게 하자. 
		cml::vector3d go_v = min(5., cml::length(b_vertices[pre_cmr_[dot_num]] - cmr_pos_last_))*cml::normalize(b_vertices[pre_cmr_[dot_num]] - cmr_pos_last_);
		cmr_pos_ += go_v; // 점차적으로 움직이게 하자. 
	}

	//cmr_pos_ = b_vertices[pre_cmr_[dot_num]];

	////////////////////////////////////////////////////////////
	// normal vector의 값이 일정 크기 이상 변해야지만 카메라의 위치가 바뀌도록 함

	/*
	double tmp = cml::length(cmr_pos_last_ - cmr_pos_now_);
	if (tmp < 10) 	cmr_pos_now_ = cmr_pos_last_;
	else			cmr_pos_now_ = cmr_pos_now_;
	*/
	//cmr_pos_ = cmr_pos_now_;

	/////////////////// Time measuring ///////////////////////////////////////////////
	g_acc_elapsed_time += milliseconds_now() - start_es_ms;
	g_count_for_e_time++;
	if (g_count_for_e_time == 100)
	{
		std::cout << "avg elapsed time = " << (g_acc_elapsed_time / g_count_for_e_time) / 1000. << std::endl;
		g_count_for_e_time = 0;
		g_acc_elapsed_time = 0;
	}
	/////////////////// Time measuring ///////////////////////////////////////////////


	return cmr_pos_;
}




cml::vector3d
BisectorSurfaceTestAni5::FindFignerSubCameraPosition()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	// BS 가 계산되지 않았을 때는 카메라의 위치를 구하지 않음
	//
	if (!b_sites.size()) return cml::vector3d();

	/*mg::Mesh mesh_joint_ = mesh_joint_obj_;
	mesh_joint_.TranslateVertices(finger_pos_);*/


	int cmr_pos_id2_ = 0;
	double min_dis = 999999999999;
	cml::vector3d env_vtx_, ptr_vtx_;
	env_vtx_.zero();
	ptr_vtx_.zero();

	//std::cout << "site : " << b_sites.size() << std::endl;
	// >> 1.
	// 두 오브젝트 사이의 거리가 가장 짧은 face를 찾음
	for (int i = 0; i < b_sites.size(); i++)
	{
		if (mesh_joint_obj_T_.vertex(b_sites[i].second - mesh_B_.num_vertices()) != finger_pos_)
			continue;

		cml::vector3d a = mesh_B_.vertex(b_sites[i].first);
		cml::vector3d b = pos_;
		double dis = cml::length(a - b);
		//std::cout << dis << " ";

		if (min_dis >= dis)
		{
			min_dis = dis;
			env_vtx_ = a;
			ptr_vtx_ = b;

			//std::cout << min_dis << " ";
			// 두 오브젝트 사이의 거리가 가장 짧은 face의 index
			cmr_pos_id2_ = i;
		}
	}

	//std::cout << std::endl;

	// >> 2.
	// 1에서 찾은 face의 중심점을 찾음
	cml::vector3d center_of_face(0, 0, 0);
	for (unsigned int i = 0; i < b_faces[cmr_pos_id2_].size(); i++)
	{
		center_of_face += b_vertices[b_faces[cmr_pos_id2_][i]];

	}
	center_of_face /= (double)b_faces[cmr_pos_id2_].size();


	// >> 3.
	// 찾은 face의 중심점은 카메라가 바라볼 center가 된다.
	//cml::vector3d cnt_pos2_ = center_of_face;


	// >> 4.
	// 카메라의 위치를 정해주자.
	//
	// 카메라의 위치 후보들을 구하자
	int *pre_cmr_tmp = new int[b_vertices.size()];
	int pre_num_ = -1;

	for (unsigned int i = 0; i < b_vertices.size(); i++)
	{
		// 처음 최대값으로 초기화
		pre_cmr_tmp[i] = b_vertices.size();
	}

	// 물체 사이의 거리를 카메라 시야를 확보할 원기둥의 지름으로 이용
	int between_length_ = cml::length(env_vtx_ - ptr_vtx_);
	between_length_ = (int)sqrt(between_length_);
	//std::cout << "between_length_ : " << between_length_ << std::endl;

	std::set<int> dup_test;
	for (unsigned int f = 0; f < b_faces.size(); f++)
	{
		for (unsigned int i = 0; i < b_faces[f].size(); i++)
		{
			// 같은 vertex는 계산하지 않음
			if (dup_test.count(b_faces[f][i]) > 0)
			{
				continue;
			}
			dup_test.insert(b_faces[f][i]);

			mg::LineSegment line_(pos_, b_vertices[b_faces[f][i]]);

			// 카메라 경로 사이에 물체에 겹치는지 확인
			bool contact = false;
			for (unsigned int j = 0; j < mesh_B_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_B_.vertex(j));

				if (dis <= between_length_)
				{
					contact = true;
				}
			}

			for (unsigned int j = 0; j < mesh_joint_obj_T_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_joint_obj_.vertex(j));

				if (dis <= between_length_)
				{
					contact = true;
				}
			}

			if (contact == false)
			{
				pre_num_++;
				pre_cmr_tmp[pre_num_] = b_faces[f][i];
			}
		}
	}

	// 찾을 수 있는 camera의 위치가 없으면 이전에 사용했던 카메라의 위치를 이용하고 함수 종료
	if (pre_num_ <= 0) {
		pos_ = finger_last_;
		return finger_last_;
	}


	// 임의로 생성했던 배열을 지움
	int *pre_cmr_ = new int[pre_num_];
	for (int i = 0; i < pre_num_; i++)
	{
		pre_cmr_[i] = pre_cmr_tmp[i];
	}
	delete[] pre_cmr_tmp;

	
	//std::cout << "pre_num : " << pre_num_ << std::endl;

	// 5.
	//
	// 보고있는 방향에서 수직인 방향에 위치한 점을 찾음
	cml::vector3d cmr_eye = cmr_->getViewDirection();
	cml::vector3d cmr_pos_now_;
	cmr_pos_now_.zero();


	double dot_min = 10000;
	int dot_num = 0;
	for (int i = 0; i < pre_num_; i++)
	{
		cml::vector3d bs_vtx_ = b_vertices[pre_cmr_[i]];
		bool check_in = CheckBoundOut_BS_vtx(bs_vtx_);

		if (check_in == true)
			continue;

		double dot = sqrt( DotProduct(cml::normalize(cmr_eye), cml::normalize(bs_vtx_)) );
		double dist = sqrt( cml::length(cmr_pos_last_ - bs_vtx_) );

		double dd = dot + 0.01*dist;

		if (dist >= between_length_ * 3)
			continue;

		if (dot_min > dd)
		{
			dot = dd;
			dot_num = i;
		}
		else
			continue;
		

	}


	cmr_pos2_ = b_vertices[pre_cmr_[dot_num]];
	//std::cout << "cmr_pos2_ : " << cmr_pos2_ << std::endl;

	////////////////////////////////////////////////////////////
	// normal vector의 값이 일정 크기 이상 변해야지만 카메라의 위치가 바뀌도록 함

	/*double tmp = cml::length(cmr_pos2_last_ - cmr_pos_now_);
	if (tmp < 20) 	cmr_pos_now_ = cmr_pos_last_;
	else			cmr_pos_now_ = cmr_pos_now_;*/

	//cmr_pos2_ = cmr_pos_now_;



	return cmr_pos2_;
}



/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//
// Leap motion function

const std::string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const std::string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };

void BisectorSurfaceTestAni5::onInit(const Leap::Controller& controller) {
	std::cout << "Initialized" << std::endl;

	finger_joint[0].zero();
	finger_joint[1].zero();
	finger_joint[2].zero();
	finger_joint[3].zero();
}

void BisectorSurfaceTestAni5::onConnect(const Leap::Controller& controller) {
	std::cout << "Connected" << std::endl;
}

void BisectorSurfaceTestAni5::onDisconnect(const Leap::Controller& controller) {
	// Note: not dispatched when running in a debugger.
	std::cout << "Disconnected" << std::endl;
}

void BisectorSurfaceTestAni5::onExit(const Leap::Controller& controller) {
	std::cout << "Exited" << std::endl;
}

void BisectorSurfaceTestAni5::onFrame(const Leap::Controller& controller) {


	// Get the most recent frame and report some basic information
	const Leap::Frame frame = controller.frame();

	Leap::HandList hands = frame.hands();
	num_leap_hands_ = hands.count();

	for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
		// Get the first hand
		const Leap::Hand hand = *hl;

		// Get the hand's normal vector and direction
		const Leap::Vector normal = hand.palmNormal();		// 손바닥의 방향
		const Leap::Vector direction = hand.direction();

		// Get fingers
		const Leap::FingerList fingers = hand.fingers();
		for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
			const Leap::Finger finger = *fl;
			//			std::cout << std::string(4, ' ') << fingerNames[finger.type()]
			//				<< " finger, id: " << finger.id()
			//				<< ", length: " << finger.length()
			//				<< "mm, width: " << finger.width() << std::endl;
			// finger type : TYPE_THUMB, TYPE_INDEX, TYPE_MIDDLE, TYPE_RING, TYPE_PINKY

			// Get finger bones
			if (fingerNames[finger.type()] == "Index")
			{
				for (int b = 0; b < 4; ++b) {
					Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
					Leap::Bone bone = finger.bone(boneType);
					//					std::cout << std::string(6, ' ') << boneNames[boneType]
					//						<< " bone, start: " << bone.prevJoint()
					//						<< ", end: " << bone.nextJoint()
					//						<< ", direction: " << bone.direction() << std::endl;

					//finger.tipPosition();

					finger_joint[b] = cml::vector3d(bone.nextJoint().x, bone.nextJoint().y, bone.nextJoint().z);
				}

				finger_pos_[0] = finger.tipPosition().x;
				finger_pos_[1] = finger.tipPosition().y;
				finger_pos_[2] = finger.tipPosition().z;
			}
		}
	}

}

void BisectorSurfaceTestAni5::onFocusGained(const Leap::Controller& controller) {
	std::cout << "Focus Gained" << std::endl;
}

void BisectorSurfaceTestAni5::onFocusLost(const Leap::Controller& controller) {
	std::cout << "Focus Lost" << std::endl;
}

void BisectorSurfaceTestAni5::onDeviceChange(const Leap::Controller& controller) {
	std::cout << "Device Changed" << std::endl;
	const Leap::DeviceList devices = controller.devices();

	for (int i = 0; i < devices.count(); ++i) {
		std::cout << "id: " << devices[i].toString() << std::endl;
		std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
		std::cout << "  isSmudged:" << (devices[i].isSmudged() ? "true" : "false") << std::endl;
		std::cout << "  isLightingBad:" << (devices[i].isLightingBad() ? "true" : "false") << std::endl;
	}
}

void BisectorSurfaceTestAni5::onServiceConnect(const Leap::Controller& controller) {
	std::cout << "Service Connected" << std::endl;
}

void BisectorSurfaceTestAni5::onServiceDisconnect(const Leap::Controller& controller) {
	std::cout << "Service Disconnected" << std::endl;
}

void BisectorSurfaceTestAni5::onServiceChange(const Leap::Controller& controller) {
	std::cout << "Service Changed" << std::endl;
}

void BisectorSurfaceTestAni5::onDeviceFailure(const Leap::Controller& controller) {
	std::cout << "Device Error" << std::endl;
	const Leap::FailedDeviceList devices = controller.failedDevices();

	for (Leap::FailedDeviceList::const_iterator dl = devices.begin(); dl != devices.end(); ++dl) {
		const Leap::FailedDevice device = *dl;
		std::cout << "  PNP ID:" << device.pnpId();
		std::cout << "    Failure type:" << device.failure();
	}
}

void BisectorSurfaceTestAni5::onLogMessage(const Leap::Controller&, Leap::MessageSeverity s, int64_t t, const char* msg) {
	switch (s) {
	case Leap::MESSAGE_CRITICAL:
		std::cout << "[Critical]";
		break;
	case Leap::MESSAGE_WARNING:
		std::cout << "[Warning]";
		break;
	case Leap::MESSAGE_INFORMATION:
		std::cout << "[Info]";
		break;
	case Leap::MESSAGE_UNKNOWN:
		std::cout << "[Unknown]";
	}
	std::cout << "[" << t << "] ";
	std::cout << msg << std::endl;
}

