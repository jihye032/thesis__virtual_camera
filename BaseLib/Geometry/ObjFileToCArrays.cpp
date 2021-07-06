
#include "BaseLib/Geometry/SolidObject.h"
#include <string>
#include <iostream>
#include <fstream>

namespace mg
{

void SolidObj2CArrays(SolidObject &obj, std::string c_file);
void Mesh2CArrays(Mesh &mesh, std::string mesh_name, std::string c_file);

void ObjFile2CArrays(std::string obj_file, std::string c_file)
{
	SolidObject obj;
	obj.ReadObjFormatFile(obj_file);

	SolidObj2CArrays(obj, c_file);
}



void SolidObj2CArrays(SolidObject &obj, std::string c_file)
{
	Mesh &mesh = *(obj.editable_mesh());
	Mesh2CArrays(mesh, obj.name(), c_file);
}

void Mesh2CArrays(Mesh &mesh, std::string mesh_name, std::string c_file)
{
	// find the greatest number of face points.
	int num_of_points = 0;
	for ( int face_id=0; face_id<mesh.num_faces(); face_id++ )
	{
		if ( num_of_points < mesh.face_size(face_id) )
		{
			num_of_points = mesh.face_size(face_id);
		}
	}

	std::ofstream fout(c_file.c_str());

	/// Write Sizes
	fout << "static const int num_vertices = " << mesh.num_vertices() << ";" << std::endl;
	fout << "static const int num_normals = "  << mesh.num_normals()  << ";" << std::endl;
	fout << "static const int num_uvs = "      << mesh.num_uvs()      << ";" << std::endl;
	fout << "static const int num_faces = "    << mesh.num_faces()    << ";" << std::endl;
	fout << std::endl;
	
	// Vertices.
	if ( mesh.vertices() > 0 )
	{
		fout << "static double vertices[" << mesh.num_vertices() << "*3] = " << "{" << std::endl;
		for ( int i=0; i<mesh.num_vertices()*3; i++ )
		{
			fout << mesh.vertices()[i];
			if ( i==mesh.num_vertices()*3-1 )
				fout << "}; ";
			else
				fout << ", ";

			if ( (i+1) % 3 == 0 ) fout << "  ";
			if ( (i+1) % 9 == 0 ) fout << std::endl;
		}
		fout << std::endl;
		fout << std::endl;
	}

	// UVs.
	if ( mesh.num_uvs() > 0 )
	{
		fout << "static double uvs[" << mesh.num_uvs() << "*2] = " << "{" << std::endl;
		for ( int i=0; i<mesh.num_uvs()*2; i++ )
		{
			fout << mesh.uvs()[i];
			if ( i==mesh.num_uvs()*2-1 )
				fout << "}; ";
			else
				fout << ", ";

			if ( (i+1) % 2 == 0 ) fout << "  ";
			if ( (i+1) % 10 == 0 ) fout << std::endl;
		}
		fout << std::endl;
		fout << std::endl;
	}

	// Normals.
	if ( mesh.num_normals() > 0 )
	{
		fout << "static double normals[" << mesh.num_normals() << "*3] = " << "{" << std::endl;
		for ( int i=0; i<mesh.num_normals()*3; i++ )
		{
			fout << mesh.normals()[i];
			if ( i==mesh.num_normals()*3-1 )
				fout << "}; ";
			else
				fout << ", ";

			if ( (i+1) % 3 == 0 ) fout << "  ";
			if ( (i+1) % 9 == 0 ) fout << std::endl;
		}
		fout << std::endl;
		fout << std::endl;
	}


	/// Group
	fout << "static char* group_name = \"" << mesh_name << "\";" << std::endl;


	/// Face
	if ( mesh.num_faces() > 0 )
	{

		// Face Sizes.
		fout << "static int face_sizes[" << mesh.num_faces() << "] = " << "{" << std::endl;
		for ( int i=0; i<mesh.num_faces(); i++ )
		{
			fout << mesh.face_sizes()[i];
			if ( i==mesh.num_faces()-1 )
				fout << "}; ";
			else
				fout << ", ";

			if ( (i+1) % 15 == 0 ) fout << std::endl;
		}
		fout << std::endl;
		fout << std::endl;

		// Face Vertex Ids.
		if ( mesh.num_vertices() > 0 )
		{
			fout << "static int face_vertex_ids[" << mesh.num_faces() << "][" << num_of_points << "] = " << "{" << std::endl;
			for ( int i=0; i<mesh.num_faces(); i++ )
			{
				fout << "{";
				for ( int j=0; j<num_of_points; j++ )
				{
					if ( j < mesh.face_size(i) )
						fout << mesh.face_vertex_ids(i)[j];
					else
						fout << -1;

					if ( j==num_of_points-1 ) 
						fout << "}";
					else
						fout << ",";

				}

				if ( i==mesh.num_faces()-1 )
					fout << "}; ";
				else
					fout << ", ";

				if ( (i+1) % 3 == 0 ) fout << std::endl;
			}
			fout << std::endl;
			fout << std::endl;
		}


		// Face Normal Ids.
		if ( mesh.num_normals() > 0 )
		{
			fout << "static int face_normal_ids[" << mesh.num_faces() << "][" << num_of_points << "] = " << "{" << std::endl;
			for ( int i=0; i<mesh.num_faces(); i++ )
			{
				fout << "{";
				for ( int j=0; j<num_of_points; j++ )
				{
					if ( j < mesh.face_size(i) )
						fout << mesh.face_normal_ids(i)[j];
					else
						fout << -1;

					if ( j==num_of_points-1 ) 
						fout << "}";
					else
						fout << ",";

				}

				if ( i==mesh.num_faces()-1 )
					fout << "}; ";
				else
					fout << ", ";

				if ( (i+1) % 3 == 0 ) fout << std::endl;
			}
			fout << std::endl;
			fout << std::endl;
		}

		// Face Uv Ids.
		if ( mesh.num_uvs() > 0 )
		{
			fout << "static int face_uv_ids[" << mesh.num_faces() << "][" << num_of_points << "] = " << "{" << std::endl;
			for ( int i=0; i<mesh.num_faces(); i++ )
			{
				fout << "{";
				for ( int j=0; j<num_of_points; j++ )
				{
					if ( j < mesh.face_size(i) )
						fout << mesh.face_uv_ids(i)[j];
					else
						fout << -1;

					if ( j==num_of_points-1 ) 
						fout << "}";
					else
						fout << ",";

				}

				if ( i==mesh.num_faces()-1 )
					fout << "}; ";
				else
					fout << ", ";

				if ( (i+1) % 3 == 0 ) fout << std::endl;
			}
			fout << std::endl;
			fout << std::endl;
		}
	
	}	// Face
	



	fout.close();
}

};
