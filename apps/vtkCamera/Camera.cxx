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
#include <sstream>
 
int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc != 9)
    {
    std::cout << "Usage: " << argv[0] << " Filename(.obj) yaw pitch roll (camera rotation [rad]) tx ty tz (camera translation [cm]) f (focal length [cm] > 0)" << std::endl;
    return EXIT_FAILURE;
    }

  // Read the mesh from (.obj) file
  std::string filename = argv[1];
  vtkSmartPointer<vtkOBJReader> reader =
    vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  // Create a mapper and actor for the mesh
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Read the extrinsic and intrinsic camera parameters
  std::istringstream ss;  
  double yaw,pitch,roll,tx,ty,tz,focalLength;
  ss.clear(); ss.str(argv[2]); ss >> yaw;         //rad
  ss.clear(); ss.str(argv[3]); ss >> pitch;       //rad
  ss.clear(); ss.str(argv[4]); ss >> roll;        //rad
  ss.clear(); ss.str(argv[5]); ss >> tx;          //cm
  ss.clear(); ss.str(argv[6]); ss >> ty;          //cm
  ss.clear(); ss.str(argv[7]); ss >> tz;          //cm
  ss.clear(); ss.str(argv[8]); ss >> focalLength; //cm

  std::cout << "camera extrinsic parameters: (" 
            << yaw << "," << pitch << "," << roll << ","
            << tx << "," << ty << "," << tz << ")" << std::endl; 

  // Define a rotation matrix to rotate the reference frame 
  // 
  //       y                z   
  //       |               /
  //       |____x   >>>   /____x
  //       /              |
  //     z/               |
  //                      y
  //
  const double refFrameRotationMatrix[]={1.0 , 0.0 , 0.0 , 0.0 ,
                                         0.0 , -1.0 , 0.0 , 0.0 ,
                                         0.0 , 0.0 , -1.0 , 0.0 ,
                                         0.0 , 0.0 , 0.0 , 1.0};
  vtkSmartPointer<vtkTransform> refFrameRotation = 
    vtkSmartPointer<vtkTransform>::New();
  refFrameRotation->SetMatrix(refFrameRotationMatrix);

  // Define the camera pose
  double cy,cp,cr,sy,sp,sr;
  cy=cos(yaw); cp=cos(pitch); sy=sin(yaw);
  sp=sin(pitch);sr=sin(roll); cr=cos(roll);
  const double cameraPoseMatrix[]={cy*cp , cy*sp*sr-sy*cr , cy*sp*cr+sy*sr , tx ,
                                   sy*cp , sy*sp*sr+cy*cr , sy*sp*cr-cy*sr , ty ,
                                   -sp   , cp*sr          , cp*cr          , tz ,
                                   0.0   , 0.0            , 0.0            , 1.0};
  vtkSmartPointer<vtkTransform> cameraPose = 
    vtkSmartPointer<vtkTransform>::New();
  cameraPose->SetMatrix(cameraPoseMatrix);
  cameraPose->Concatenate(refFrameRotation); // Rotate the reference frame
  cameraPose->PreMultiply();

  // Set the camera position and focal point according to the defined camera pose
  vtkSmartPointer<vtkCamera> camera = 
    vtkSmartPointer<vtkCamera>::New();
  camera->SetPosition(0,0,0);
  camera->SetFocalPoint(0,0,-focalLength);
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
