
#pragma once


namespace mg
{
	class Mesh;
	class SolidObject;
	class PrimitiveShape;
	class PrimitiveCapsule;
	class PrimitiveBox;
	class PrimitiveSphere;
	class PrimitiveCylinder;
	class PrimitiveComposition;

	void DrawMesh(const Mesh& mesh);
	void DrawSolidObject(const SolidObject& obj);
	void DrawPrimitiveShape(const PrimitiveShape& p);
	void DrawPrimitiveCapsule(const PrimitiveCapsule& p);
	void DrawPrimitiveCylinder(const PrimitiveCylinder& p);
	void DrawPrimitiveBox(const PrimitiveBox& p);
	void DrawPrimitiveSphere(const PrimitiveSphere& p);
	void DrawPrimitiveComposition(const PrimitiveComposition& p);
};