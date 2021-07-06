#include "BisectorSurfaceTestAni.h"
#include "BaseLib/GL4U/GL_ResourceManager.h"
#include "BaseLib/FLTKU/AnimationViewer.h"
#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "FL/fl.h"
#include <iostream>
#include <queue>



BisectorSurfaceTestAni::BisectorSurfaceTestAni()
{
	camera1_.lookAt({ 300., 300., 300. }, { 0., 0., 0. }, { 0., 1., 0. });
	final_ani_time_ms(2000);


	// box obj
	scene_root_ = std::make_unique<mg::GL_SceneRoot>();
	mg::GL_ResourceManager::singleton()->ImportByAssimp(scene_root_.get(), "../Data/obj/cup2.obj", "cup");
	scene_root_->BuildSinglePolygonMesh(&mesh_B_);
	mesh_B_.ScaleUniformlyVertices(70);
	mesh_B_.TranslateVertices(cml::vector3d(0, 100, 0));
	mesh_B_.Triangulate();



	// pointer
	//
	// 직접 정해준 pointer의 위치
	cml::vector3d joint_pos = cml::vector3d(0, 80, 0);


	finger_joint[0].zero();
	finger_joint[1].zero();
	finger_joint[2] = joint_pos;


	// 계산에 사용되는 pointer box
	//mesh_joint_.CreateBox(10, 10, 10);
	//mesh_joint_.TranslateVertices(joint_pos);


	// pointer obj -> 그림만
	mg::Mesh b, c, d, e, f, g;
	mesh_joint_obj_.CreateCapsule(180, 30);
	mesh_joint_obj_.TranslateVertices(cml::vector3d(0, 180, 0));

	b.CreateCylinder(20, 30);
	c.CreateCylinder(20, 30);
	d.CreateCylinder(20, 30);
	e.CreateCylinder(20, 30);
	f.CreateCylinder(20, 30);
	g.CreateCylinder(20, 30);

	b.TranslateVertices(cml::vector3d(0, 80, 0));
	c.TranslateVertices(cml::vector3d(0, 95, 0));
	d.TranslateVertices(cml::vector3d(0, 110, 0));
	e.TranslateVertices(cml::vector3d(0, 125, 0));
	f.TranslateVertices(cml::vector3d(0, 150, 0));
	g.TranslateVertices(cml::vector3d(0, 180, 0));

	//mesh_joint_obj_.Merge(b);
	//mesh_joint_obj_.Merge(c);
	mesh_joint_obj_.Merge(d);
	mesh_joint_obj_.Merge(e);
	//esh_joint_obj_.Merge(f);
	mesh_joint_obj_.Merge(g);

	mesh_joint_obj_.TranslateVertices(cml::vector3d(17, 0, 0));
	mesh_joint_obj_.Triangulate();




	// camera position
	//
	cmr_pos_ = joint_pos;
	cmr_pos_last_.zero();
	cnt_pos_.zero();


	// 카메라의 위치를 알려주기 위한 원
	mesh_cmr_.CreateSphere(6);
	mesh_cmr_.TranslateVertices(cmr_pos_);
	mesh_cmr_.Triangulate();


	// leap motion 손의 개수
	//
	// leap motion을 현재는 사용하지 않기 때문에 0
	num_leap_hands_ = 0;
	findsubcmr_ = false;
}

void
BisectorSurfaceTestAni::SetCurrentTime(mg::AniTime_ms t)
{
	if (t<0) cur_time_ms_ = 0;
	//else if (t >= final_ani_time_ms_) cur_time_ms_ = final_ani_time_ms_;
	else cur_time_ms_ = t;

	//mg::AniTime_ms pre_time_ms = cur_time_ms_;


	static int frmae_id = -1;
	if (frmae_id == GetCurFrameId()) return;

	frmae_id = GetCurFrameId();


	if (num_leap_hands_ == 0)
	{
		cml::vector3d cmr_pos(cmr_pos_);

		// set user camera
		//
		cml::vector3d eye(cmr_pos);
		cml::vector3d center(cnt_pos_);
		cml::vector3d up(cml::vector3d(0, 1, 0));
		camera1_.lookAt(eye, center, up);

		num_leap_hands_ = 1;
	}

	else if (num_leap_hands_ >= 1)
	{
		if (findsubcmr_ == false)
		{
			FindPreSubCamera();


			if (pre_num_ >0)
				findsubcmr_ = true;
		}


		// set user camera
		//
		cml::vector3d cmr_pos = FindSubCameraPosition();
		cml::vector3d eye(cmr_pos);
		cml::vector3d center(cnt_pos_);
		cml::vector3d up(cml::vector3d(0, 1, 0));
		camera1_.setFov(cml::pi()/2);
		camera1_.lookAt(eye, center, up);


		b_surface_.SetVertices(mesh_B_.num_vertices(), mesh_B_.vertices(),
			mesh_joint_obj_.num_vertices(), mesh_joint_obj_.vertices());
		b_surface_.Update();


	}



	NotifyAnimationUpdated();
}


void
BisectorSurfaceTestAni::Draw(mg::AnimationViewer *w)
{
	int win_w = w->w();
	int win_h = w->h();


	//renderer_->SetModelIdentity();
	renderer_->UseDefaultCamera();
	DrawInSubViewport(0, 0, win_w, win_h, 0);
	cmr_ = renderer_->camera();



	// Right-Top PIP
	renderer_->UseCamera(&camera1_);
	DrawInSubViewport2(win_w - win_w / 3, win_h - win_h / 3, win_w / 3, win_h / 3, 3);

	renderer_->UseDefaultCamera();

}

void
BisectorSurfaceTestAni::DrawInSubViewport(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
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


	renderer_->SetModelIdentity();

	renderer_->SetColor(0);
	renderer_->DrawMesh(&mesh_B_);


	//if (flag_draw_bisector_)
	{
		renderer_->SetColor(7);
		renderer_->DrawMesh(&mesh_joint_obj_);


		renderer_->SetColor(0, 1, 0);
		renderer_->DrawSphere(8, cmr_pos_);

		// Closed Position - Red
		renderer_->SetColor(1, 0, 0);
		renderer_->DrawSphere(8, cnt_pos_);

		if (0) {
			glPointSize(4);
			renderer_->SetColor(20);
			renderer_->glBegin(GL_POINTS);
			for (int i = 0; i < b_vertices.size(); i++)
			{
				renderer_->glVertex(b_vertices[i]);
			}
			renderer_->glEnd();
		}


		renderer_->EnableTransparency(true);
		renderer_->SetColor(3);
		DrawBisector();
		renderer_->EnableTransparency(false);
	}

	glDisable(GL_SCISSOR_TEST);
}


void
BisectorSurfaceTestAni::DrawInSubViewport2(int vp_x, int vp_y, int vp_w, int vp_h, int border_width = 0)
{
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

	renderer_->SetColor(0);
	renderer_->DrawMesh(&mesh_B_);

	renderer_->SetColor(2);
	renderer_->DrawMesh(&mesh_joint_obj_);


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
BisectorSurfaceTestAni::DrawBisector()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	///////////////////////////////////////////////////////////////////////////////
	// sites
	if (false)
	{
		renderer_->SetColor(0, 0, 0);
		renderer_->glBegin(GL_LINES);
		for (unsigned int i = 0; i < b_sites.size() && i < b_faces.size(); i++)
		{
			// Boundary Check
			bool out_bound = false;
			for (int fv_i = 0; fv_i < (int)b_faces[i].size(); fv_i++)
			{
				if (CheckBoundOut(b_vertices[b_faces[i][fv_i]]))
				{
					out_bound = true;
				}
			}
			if (out_bound) continue;

			renderer_->glVertex(b_surface_.GetVertexOfA(b_sites[i].first));
			renderer_->glVertex(b_surface_.GetVertexOfB(b_sites[i].second));
		}
		renderer_->glEnd();
	}


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
		/*if ( std::find(b_faces[i].begin(), b_faces[i].end(), 0) != b_faces[i].end() )
		{
		continue;
		}*/

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

cml::vector3d
BisectorSurfaceTestAni::FindPreSubCamera()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	// BS 가 계산되지 않았을 때는 카메라의 위치를 구하지 않음
	//
	if (!b_sites.size()) return cml::vector3d();



	double min_dis = 99999999999999999;
	cml::vector3d env_vtx_, ptr_vtx_;
	env_vtx_.zero();
	ptr_vtx_.zero();

	// >> 1.
	// 두 오브젝트 사이의 거리가 가장 짧은 face를 찾음
	for (int i = 0; i < b_sites.size(); i++)
	{
		cml::vector3d a = mesh_B_.vertex(b_sites[i].first);
		cml::vector3d b = mesh_joint_.vertex(b_sites[i].second);
		double dis = cml::length(a - b);

		if (min_dis > dis)
		{
			min_dis = dis;
			env_vtx_ = a;
			ptr_vtx_ = b;

			// 두 오브젝트 사이의 거리가 가장 짧은 face의 index
			cmr_pos_id_ = i;
		}
	}


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
	pre_num_ = -1;

	for (unsigned int i = 0; i < b_vertices.size(); i++)
	{
		// 처음 최대값으로 초기화
		pre_cmr_tmp[i] = b_vertices.size();
	}

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

			mg::LineSegment line_(cnt_pos_, b_vertices[b_faces[f][i]]);

			// 카메라 경로 사이에 물체에 겹치는지 확인
			bool contact = false;
			for (unsigned int j = 0; j < mesh_B_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_B_.vertex(j));

				if (dis <= 3)
				{
					contact = true;
				}
			}

			for (unsigned int j = 0; j < mesh_joint_.num_vertices(); j++)
			{
				double dis = line_.MinDistToPoint(mesh_joint_.vertex(j));

				if (dis <= 3)
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

	// 임의로 생성했던 배열을 지움
	pre_cmr_ = new int[pre_num_];
	for (int i = 0; i < pre_num_; i++)
	{
		pre_cmr_[i] = pre_cmr_tmp[i];
	}
	delete[] pre_cmr_tmp;



	std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
	std::cout << " cmr_ num : " << pre_num_ << std::endl;
	std::cout << " bs_ size : " << b_vertices.size() << std::endl;
	std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;



	//int tmp_cmr = rand() % (pre_num_);
	//cmr_pos_now_ = b_vertices[pre_cmr_[tmp_cmr]];

	return cml::vector3d();
}

cml::vector3d
BisectorSurfaceTestAni::FindSubCameraPosition()
{
	const std::vector<cml::vector3d>&      b_vertices = b_surface_.voronoi_vertices();
	const std::vector< std::vector<int> >& b_faces = b_surface_.bisector_faces();
	const std::vector<cml::vector3d>&      b_normals = b_surface_.bisector_faces_normals();
	const std::vector< std::pair<int, int> >&      b_sites = b_surface_.bisector_faces_sites();


	// BS 가 계산되지 않았을 때는 카메라의 위치를 구하지 않음
	//
	if (!b_sites.size()) return cml::vector3d();



	cml::vector3d cmr_pos_now_;
	cmr_pos_now_.zero();


	// 5.
	//
	// 보고있는 방향에서 수직인 방향에 위치한 점을 찾음
	cml::vector3d cmr_eye = cmr_->getViewDirection();
	//cml::vector3d cmr_cnt = cnt_pos_;
	//cml::vector3d cmr_nor = cmr_eye - cmr_cnt;

	double dot = 10000;
	int dot_num = 0;
	for (int i = 0; i < pre_num_ - 1; i++)
	{
		cml::vector3d tmp = b_vertices[pre_cmr_[i]];

		double dd = DotProduct(cmr_eye, tmp);

		if (dd >= 0)
		{
			if (dot > dd)
			{
				dot = dd;
				dot_num = i;
			}
			else
				continue;
		}
		else {
			dd = dd * -1;

			if (dot > dd)
			{
				dot = dd;
				dot_num = i;
			}
			else
				continue;
		}

	}
	std::cout << " dot_ num : " << dot_num << std::endl;
	std::cout << " cmr_ num : " << pre_cmr_[dot_num] << std::endl;
	cmr_pos_now_ = b_vertices[pre_cmr_[dot_num]];




	std::cout << " cmr_pos_now_ : " << cmr_pos_now_ << std::endl;
	//std::cout << " center_ pos : " << center_of_face << std::endl;
	std::cout << "----------------------------------------------------------\n" << std::endl;



	////////////////////////////////////////////////////////////
	// normal vector의 값이 일정 크기 이상 변해야지만 카메라의 위치가 바뀌도록 함
	/*
	double tmp = pow(cmr_pos_last_[0] - cmr_pos_now_[0], 2)
	+ pow(cmr_pos_last_[1] - cmr_pos_now_[1], 2)
	+ pow(cmr_pos_last_[2] - cmr_pos_now_[2], 2);

	if (tmp < 10) 	cmr_pos_now_ = cmr_pos_last_;
	else			cmr_pos_now_ = cmr_pos_now_;
	*/

	cmr_pos_last_ = cmr_pos_now_;
	cmr_pos_ = cmr_pos_now_;



	return cmr_pos_;
}


/*
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

void BisectorSurfaceTestAni::onInit(const Leap::Controller& controller) {
std::cout << "Initialized" << std::endl;

finger_joint[0].zero();
finger_joint[1].zero();
finger_joint[2].zero();

finger_joint_p[0].zero();
finger_joint_p[1].zero();
finger_joint_p[2].zero();
}

void BisectorSurfaceTestAni::onConnect(const Leap::Controller& controller) {
std::cout << "Connected" << std::endl;
}

void BisectorSurfaceTestAni::onDisconnect(const Leap::Controller& controller) {
// Note: not dispatched when running in a debugger.
std::cout << "Disconnected" << std::endl;
}

void BisectorSurfaceTestAni::onExit(const Leap::Controller& controller) {
std::cout << "Exited" << std::endl;
}

void BisectorSurfaceTestAni::onFrame(const Leap::Controller& controller) {


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
for (int b = 0; b < 4; ++b) {
Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(b);
Leap::Bone bone = finger.bone(boneType);
//					std::cout << std::string(6, ' ') << boneNames[boneType]
//						<< " bone, start: " << bone.prevJoint()
//						<< ", end: " << bone.nextJoint()
//						<< ", direction: " << bone.direction() << std::endl;


// 검지 손가락만 골라내기
if (fingerNames[finger.type()] == "Index" && b != 3)
{
finger_joint[b] = cml::vector3d(bone.nextJoint().x, bone.nextJoint().y, bone.nextJoint().z);
//std::cout << "finger_joint : " << finger_joint[b] << std::endl;
}

}

}
}

}

void BisectorSurfaceTestAni::onFocusGained(const Leap::Controller& controller) {
std::cout << "Focus Gained" << std::endl;
}

void BisectorSurfaceTestAni::onFocusLost(const Leap::Controller& controller) {
std::cout << "Focus Lost" << std::endl;
}

void BisectorSurfaceTestAni::onDeviceChange(const Leap::Controller& controller) {
std::cout << "Device Changed" << std::endl;
const Leap::DeviceList devices = controller.devices();

for (int i = 0; i < devices.count(); ++i) {
std::cout << "id: " << devices[i].toString() << std::endl;
std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
std::cout << "  isSmudged:" << (devices[i].isSmudged() ? "true" : "false") << std::endl;
std::cout << "  isLightingBad:" << (devices[i].isLightingBad() ? "true" : "false") << std::endl;
}
}

void BisectorSurfaceTestAni::onServiceConnect(const Leap::Controller& controller) {
std::cout << "Service Connected" << std::endl;
}

void BisectorSurfaceTestAni::onServiceDisconnect(const Leap::Controller& controller) {
std::cout << "Service Disconnected" << std::endl;
}

void BisectorSurfaceTestAni::onServiceChange(const Leap::Controller& controller) {
std::cout << "Service Changed" << std::endl;
}

void BisectorSurfaceTestAni::onDeviceFailure(const Leap::Controller& controller) {
std::cout << "Device Error" << std::endl;
const Leap::FailedDeviceList devices = controller.failedDevices();

for (Leap::FailedDeviceList::const_iterator dl = devices.begin(); dl != devices.end(); ++dl) {
const Leap::FailedDevice device = *dl;
std::cout << "  PNP ID:" << device.pnpId();
std::cout << "    Failure type:" << device.failure();
}
}

void BisectorSurfaceTestAni::onLogMessage(const Leap::Controller&, Leap::MessageSeverity s, int64_t t, const char* msg) {
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

*/