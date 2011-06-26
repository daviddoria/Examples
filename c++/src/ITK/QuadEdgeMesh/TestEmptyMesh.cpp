#include "itkQuadEdgeMesh.h"

#include "itkLineCell.h"

const unsigned int Dimension = 3;
typedef itk::QuadEdgeMesh< float, Dimension >   MeshType;
	
void Works();
void DoesntWork();

int main(int, char *[])
{
	//Works();
	DoesntWork();
	return 0;
}


void Works()
{
	std::cout << "Works..." << std::endl;
	MeshType::Pointer  mesh = MeshType::New();
	std::cout << "Points = " << mesh->GetNumberOfPoints() << std::endl;

}

void DoesntWork()
{
	std::cout << "DoesntWork..." << std::endl;
	MeshType* mesh = MeshType::New();
	std::cout << "Points = " << mesh->GetNumberOfPoints() << std::endl;
}
