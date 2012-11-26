#include <vtkSmartPointer.h>
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
#include <vtkCamera.h>
#include <vtkMath.h>
#include <vtkTransform.h>
#include <vtkAxesActor.h>
#include <string>
#include <ostream>
 
int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc != 2)
    {
    std::cout << "Usage: " << argv[0] << " Filename(.obj)" << std::endl;
    return EXIT_FAILURE;
    }
 
  std::string filename = argv[1];
  vtkSmartPointer<vtkOBJReader> reader =
    vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
 
  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Define the 3 camera poses (front view, right view, left view)
  const double cameraPose0[]={-1.0,0.0,0.0,0.0,
                              0.0,1.0,0.0,0.0,
                              0.0,0.0,-1.0,-200.0,
                              0.0,0.0,0.0,1.0};
  const double cameraPose1[]={0.0,0.0,-1.0,-200.0,
                              0.0,1.0,0.0,0.0,
                              1.0,0.0,0.0,0.0,
                              0.0,0.0,0.0,1.0};
  const double cameraPose2[]={0.0,0.0,1.0,200.0,
                              0.0,1.0,0.0,0.0,
                             -1.0,0.0,0.0,0.0,
                              0.0,0.0,0.0,1.0};
  vtkSmartPointer<vtkTransform> cameraPose = 
    vtkSmartPointer<vtkTransform>::New();
  cameraPose->SetMatrix(cameraPose0);

  double focalLength = 5; //in cm
  vtkSmartPointer<vtkCamera> camera = 
    vtkSmartPointer<vtkCamera>::New();
  camera->SetPosition(0,0,focalLength);
  camera->SetFocalPoint(0,0,0);
  camera->ApplyTransform(cameraPose);

  // Print out the camera configuration to the console
  std::ostream objOstream (cout.rdbuf());
  camera->Print(objOstream);

  // Add the world axes
  vtkSmartPointer<vtkAxesActor> axes =
    vtkSmartPointer<vtkAxesActor>::New();
  axes->SetTotalLength(20.0,20.0,20.0); 	

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
 
  renderer->SetActiveCamera(camera);
  renderer->AddActor(axes);

  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600,600);
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  // Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(0,0,0); // Background color black
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}
