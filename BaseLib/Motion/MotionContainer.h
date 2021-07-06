
#pragma once

#include "ml.h"
#include <string>
#include <map>
#include <fstream>

namespace ml
{

typedef std::vector<Motion*> MotionPList;

class MotionContainer
{
public:
	MotionContainer();

	void addMotionBvhFile(std::string motion_file, std::string name="", double scale=1.0);
	void addMotionBvhSetDir(std::string dir_path, std::string prefix_of_file, double scale=1.0, unsigned int max_byte=0, int num=0);
	virtual void addMotionDir(std::string asf_file, std::string dir_path, std::string prefix_of_file, unsigned int max_byte=0, int max_file_num=0);
	virtual void addMotionFile(std::string asf_file, std::string motion_file, std::string name="");
	virtual void addMotion(Motion *motion, std::string name);
	virtual void addMotion(Motion *motion);

	bool isExist(Motion *m);
	bool isExist(std::string name);

	std::string getName(int i);
	std::string getName(Motion *m);
	const Motion* getMotion(int i);
	const Motion *getMotion(std::string name);
	Motion* getEditableMotion(int i);
	Motion* getEditableMotion(std::string name);

	unsigned int CountTotalFrames() const;
	
	//////////////////////////////////////////////////////////////////////
	// std-style iterator
	typedef MotionPList::iterator iterator;
	typedef MotionPList::const_iterator const_iterator;
	unsigned int size() const { return motions_.size(); }
	iterator begin() { return motions_.begin(); }
	const_iterator begin() const { return motions_.cbegin(); }
	iterator end() { return motions_.end(); }
	const_iterator end() const { return motions_.cend(); }
	const_iterator cbegin() const { return motions_.cbegin(); }
	const_iterator cend() const { return motions_.cend(); }


	//////////////////////////////////////////////////////////////////////
	// std-style iterator for all motion-frames
	class frame_iter;
	
	class frame_iter : public std::iterator<std::input_iterator_tag, MotionFrame, long, MotionFrame*, MotionFrame&>
	{
	private:
		MotionContainer *mc_;
		MotionContainer::iterator mc_iter_;
		MotionFrame motion_frame_;

	public:
		frame_iter(MotionContainer *mc, MotionContainer::iterator mc_iter, int frame_id) : 
			mc_(mc), mc_iter_(mc_iter)
		{ 
			if ( mc_iter_ == mc_->end() )
			{
				motion_frame_.motion(nullptr);
			}
			else
			{
				motion_frame_.motion(*mc_iter_);
			}
			motion_frame_.frame_id(frame_id);
		}

		frame_iter operator++() 
		{ 
			if ( mc_ == nullptr ) return *this;

			int cur_f = motion_frame_.frame_id();
			cur_f++;

			if ( cur_f >= (int)(*mc_iter_)->size() )
			{
				motion_frame_.frame_id(0);
				// next motion
				mc_iter_++;
				if ( mc_iter_ == mc_->end() )
				{
					motion_frame_.motion(nullptr);
				}
				else
				{
					motion_frame_.motion(*mc_iter_);
				}
			}
			else
			{
				motion_frame_.frame_id(cur_f);
			}

			return *this; 
		}

		reference operator*() { return motion_frame_; }
		pointer operator->() { return &(motion_frame_); }
		bool operator==(const frame_iter& rhs) { return (motion_frame_==rhs.motion_frame_); }
		bool operator!=(const frame_iter& rhs) { return (motion_frame_!=rhs.motion_frame_); }
	};


	frame_iter frame_begin() 
	{ 
		frame_iter i(this, motions_.begin(), 0); 
		return i; 
	}

	frame_iter frame_end() 
	{
		frame_iter i(this, motions_.end(), 0); 
		return i; 
	}

	unsigned int frame_size() const
	{
		return CountTotalFrames();
	}


	//////////////////////////////////////////////////////////////////////
	// I/O
	void saveBinary(std::ofstream &out);
	void loadBinary(std::ifstream &in);


protected:
	std:: string getFilenameFromPath(std::string path);


protected:
	std::vector<Motion*> motions_;
};



//////////////////////////////////////////////////////////////////////
// Algorithms
typedef std::vector<MotionFrame> MotionKeyFrames;
void SelectKeyFramesByKineticEnergy(const MotionContainer *mc , MotionKeyFrames &out_key_frames);


};