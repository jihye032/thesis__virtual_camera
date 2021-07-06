#pragma once

namespace mg
{
	/**
	mg::InitBaseLib() must be called before InitFlGlew();
	*/
	void InitBaseLib(char *resource_path="../BaseLib/Resources/");

	/**
	the path includes slash at the end.
	*/
	const char* GetBaseLibResourcePath();
}