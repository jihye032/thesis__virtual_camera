#include "BaseLib/BaseLib.h"
#include <string>

static std::string g_resource_path = "../BaseLib/Resources/";

namespace mg
{

	void InitBaseLib(char *resource_path)
	{
		if ( resource_path == nullptr )
		{
			g_resource_path = "/";
		}

		else 
		{
			g_resource_path = resource_path;
			if ( g_resource_path.back() != '/' ) g_resource_path += "/";
		}
	}

	const char* GetBaseLibResourcePath()
	{
		return g_resource_path.c_str();
	}

}