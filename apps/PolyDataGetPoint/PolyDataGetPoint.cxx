#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOBJReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <string>


int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc != 2)
    {
    std::cout << "Usage: " << argv[0] << " Filename(.obj)" << std::endl;
    return EXIT_FAILURE;
    }
 
  // Read the mesh from file
  std::string filename = argv[1];
  vtkSmartPointer<vtkOBJReader> reader =
    vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 
  vtkPolyData* polydata = mapper->GetInput();
 
  // Write all of the coordinates of the points in the vtkPolyData to the console.
  for(vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i++)
    {
    double p[3];
    polydata->GetPoint(i,p);
    // This is identical to:
    // polydata->GetPoints()->GetPoint(i,p);
    std::cout << "Point " << i << " : (" << p[0] << " " << p[1] << " " << p[2] << ")" << std::endl;
    }
 
  return EXIT_SUCCESS;
}
