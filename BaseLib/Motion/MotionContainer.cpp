
#include <algorithm>
#ifdef WIN32
#include <windows.h>
#endif
#include "ml.h"
#include <sstream>


namespace ml
{

MotionContainer::MotionContainer()
{
}






std::string 
MotionContainer::getName(int i)
{
	if ( (int)motions_.size() <= i ) return "";
	return motions_[i]->name();
}

std::string 
MotionContainer::getName(Motion *m)
{
	int ii=0;

	MotionPList::iterator iter;
	for ( iter=motions_.begin(); iter!=motions_.end(); iter++ )
	{
		if ( m == (*iter)) return (*iter)->name();
	}

	return "";
}

const Motion* 
MotionContainer::getMotion(int i)
{
	return getEditableMotion(i);
}


const Motion*
MotionContainer::getMotion(std::string name)
{
	return getEditableMotion(name);
}


Motion* 
MotionContainer::getEditableMotion(int i)
{
	if ( (int)motions_.size() <= i ) return nullptr;
	return motions_[i];
}

Motion*
MotionContainer::getEditableMotion(std::string name)
{
	MotionPList::iterator iter;
	for ( iter=motions_.begin(); iter!=motions_.end(); iter++ )
	{
		if ( name.compare((*iter)->name()) == 0 ) return (*iter);
	}
	return nullptr;
}


bool
MotionContainer::isExist(std::string name)
{
	MotionPList::iterator iter;
	for ( iter=motions_.begin(); iter!=motions_.end(); iter++ )
	{
		if ( name.compare((*iter)->name()) == 0 ) return true;
	}

	return false;
}


bool
MotionContainer::isExist(Motion *m)
{
	MotionPList::iterator iter;
	for ( iter=motions_.begin(); iter!=motions_.end(); iter++ )
	{
		if ( m == (*iter) ) return true;
	}

	return false;
}

unsigned int
MotionContainer::CountTotalFrames() const
{
	int t=0;
	for ( auto m : motions_ )
	{
		t += m->size();
	}

	return t;
}


void 
MotionContainer::addMotion(Motion *motion)
{
	motions_.push_back(motion);
}

void 
MotionContainer::addMotion(Motion *motion, std::string name)
{
	motion->name(name);
	motions_.push_back(motion);
}



std::string
MotionContainer::getFilenameFromPath(std::string path)
{
	std::string::size_type end_i = path.rfind(".")-1;
	std::string::size_type start_i =  path.rfind("/")+1;
	return path.substr(start_i, end_i-start_i+1);
}



void MotionContainer::addMotionBvhFile(std::string motion_file, std::string name, double scale)
{
	BVHReader bvh_reader;
	Motion *motion = new Motion;
	//bvh_reader.OpenBVH(*motion, motion_file, scale, true);
	bvh_reader.LoadBVH(motion_file.c_str(), motion, true, scale);

	/*if ( true )
	{
		Motion *r_motion = new Motion(motion->getBody());
		r_motion->resampling(motion->getSize()/8, *motion);
		delete motion;
		motion = r_motion;
	}*/

	if ( (int)name.size()==0 )
		name = getFilenameFromPath(motion_file);

	addMotion(motion, name);


	printf("%s, %s\n", name.c_str(), motion_file.c_str());

}

void MotionContainer::addMotionBvhSetDir(std::string dir_path, std::string prefix_of_file, double scale, unsigned int max_byte, int num)
{

#ifdef WIN32
	WIN32_FIND_DATAA find_file_data;
	HANDLE h_find;

	if ( *(dir_path.end()-1) != '/' )
		dir_path.append("/");

	std::string reg_exp = dir_path;
	if ( (int)prefix_of_file.size() > 0 )
		reg_exp.append(prefix_of_file);

	reg_exp.append("*.bvh");

	int count = 0;

	h_find = FindFirstFileA(reg_exp.c_str(), &find_file_data);		
	if ( h_find != INVALID_HANDLE_VALUE )
	{
		do
		{
			if ( max_byte == 0
				|| 
				( find_file_data.nFileSizeHigh == 0
				&&
				max_byte > find_file_data.nFileSizeLow )
				)
			{
				std::string file = dir_path + find_file_data.cFileName;
				//std::basic_string <char>::size_type f = file.find(prefix_of_file);
				addMotionBvhFile(file, "", scale);

				count++;

				if ( num == count ) break;
			}
		}
		while ( FindNextFileA(h_find, &find_file_data) );

	}
	FindClose(h_find);
#endif
}

void 
MotionContainer::addMotionFile(std::string asf_file, std::string motion_file, std::string name)
{
	AMCReader amc_reader;
	//PmHuman *human = new PmHuman(asf_file.c_str());
	Motion *motion = new Motion;//(human);
	//motion->openAMC_Woody(motion_file.c_str());
	amc_reader.LoadAMC(motion_file.c_str(), asf_file.c_str(), motion, true, 1.0);

	if ( (int)name.size()==0 )
		name = getFilenameFromPath(motion_file);

	addMotion(motion, name);

	printf("%s, %s\n", name.c_str(), motion_file.c_str());

}


void 
MotionContainer::addMotionDir(std::string asf_file, std::string dir_path, std::string prefix_of_file, unsigned int max_byte, int max_file_num)
{
	// PmHuman *human = new PmHuman(asf_file.c_str());

#ifdef WIN32
	WIN32_FIND_DATAA find_file_data;
	HANDLE h_find;

	if ( *(dir_path.end()-1) != '/' )
		dir_path.append("/");

	std::string reg_exp = dir_path;
	if ( (int)prefix_of_file.size() > 0 )
		reg_exp.append(prefix_of_file);

	reg_exp.append("*.amc");

	int count = 0;

	h_find = FindFirstFileA(reg_exp.c_str(), &find_file_data);		
	if ( h_find != INVALID_HANDLE_VALUE )
	{
		do
		{
			if ( max_byte <= 0
				|| 
				( find_file_data.nFileSizeHigh == 0
				&&
				max_byte > find_file_data.nFileSizeLow )
				)
			{
				std::string file = dir_path + find_file_data.cFileName;
				addMotionFile(asf_file, file);

				count++;

				if ( max_file_num > 0 && max_file_num <= count ) break;
			}
		}
		while ( FindNextFileA(h_find, &find_file_data) );

	}
	FindClose(h_find);
#endif
}


void
MotionContainer::saveBinary(std::ofstream &out)
{
	std::cerr << " Not Implmented Yet!" << "- MotionContainer::saveBinary(std::ofstream &out)" << std::endl ;
	std::exit(0);
	/*
	int data_size = (int)motion_map.size();
	out.write((char*)&data_size, sizeof(data_size));

	MotionNameMap::iterator iter;
	for ( iter=this->motion_map.begin(); iter!=motion_map.end(); iter++ )
	{
		int name_len = iter->first.size();
		out.write((char*)&name_len, sizeof(name_len));
		out.write((char*)iter->first.c_str(), iter->first.size());
		iter->second->saveBinary(out);
	}
	*/
}

void
MotionContainer::loadBinary(std::ifstream &in)
{
	std::cerr << " Not Implmented Yet!" << "- MotionContainer::loadBinary(std::ifstream &in)" << std::endl ;
	std::exit(0);
	/*
	motion_map.clear();

	int data_size;
	in.read((char*)&data_size, sizeof(data_size));

	for ( int i=0; i<data_size; i++ )
	{
		int name_len;
		in.read((char*)&name_len, sizeof(name_len));
		char name[256];
		in.read((char*)name, name_len);
		name[name_len] = '\0';

		printf("%d, %s\n", name_len, name);

		Motion *motion = new Motion;
		motion->loadBinary(in);

		//std::string name_str = name;
		motion_map[name] = motion;
	}
	*/
}






//////////////////////////////////////////////////////////////////////
// Algorithms
void SelectKeyFramesByKineticEnergy(const MotionContainer *mc , MotionKeyFrames &out_key_frames)
{
	for ( Motion* m : *mc )
	{
		std::vector<int> key_frames;
		SelectKeyFramesByKineticEnergy(m, key_frames);

		for ( int f : key_frames )
		{
			out_key_frames.push_back({m, f});
		}
	}
}




};
