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
#include <Eigen/Core> // Eigen basic types and functions
#include <Eigen/LU> // Eigen matrix inversion
#include <string>
#include <ostream>
#include <sstream>

#include <vtkSphereSource.h>
#include "vtkProperty.h" 
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include "vtkTextActor3D.h"
#include "vtkVectorText.h" 
#include "vtkLinearExtrusionFilter.h"


void display3DPointAndIndex(const vtkSmartPointer<vtkRenderer> & renderer,
                            const double x,const double y,const double z,unsigned int pIdx)
{
    double sphereRadius = 1.0;
    vtkSmartPointer<vtkSphereSource> sphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(x,y,z);
    sphereSource->SetRadius(sphereRadius);  

    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
    
    vtkSmartPointer<vtkActor> sphereActor = 
      vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(0,0,1); // sphere color blue 

    renderer->AddActor(sphereActor);

    std::stringstream ss;
    ss << pIdx;
    vtkVectorText* vecText = vtkVectorText::New();
    vecText->SetText(ss.str().c_str());

    vtkLinearExtrusionFilter* extrude = vtkLinearExtrusionFilter::New();
    extrude->SetInputConnection( vecText->GetOutputPort());
    extrude->SetExtrusionTypeToNormalExtrusion();
    extrude->SetVector(0, 0, 1 );
    extrude->SetScaleFactor (0.5);

    vtkPolyDataMapper* txtMapper = vtkPolyDataMapper::New();
    txtMapper->SetInputConnection( extrude->GetOutputPort());
    vtkActor* txtActor = vtkActor::New();
    txtActor->SetMapper(txtMapper);
    txtActor->SetPosition(x+sphereRadius,y,z-sphereRadius);
    txtActor->SetOrientation(180,0,0);
    txtActor->GetProperty()->SetColor(0,0,1);
    renderer->AddActor(txtActor);

}

int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc != 10)
    {
    std::cout << "Usage: " << argv[0] << " Filename(.obj) yaw pitch roll (camera rotation [rad]) tx ty tz (camera translation [cm]) f (focal length [cm] > 0) r (pixel/cm)" << std::endl;
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

  // Get the mesh data
  vtkPolyData* polydata = mapper->GetInput();
  std::cout << "Number of vertices: " << polydata->GetNumberOfPoints() << std::endl;

  // Read the extrinsic and intrinsic camera parameters
  std::istringstream ss;  
  double yaw,pitch,roll,tx,ty,tz,focalLength,ratio;
  ss.clear(); ss.str(argv[2]); ss >> yaw;         //rad
  ss.clear(); ss.str(argv[3]); ss >> pitch;       //rad
  ss.clear(); ss.str(argv[4]); ss >> roll;        //rad
  ss.clear(); ss.str(argv[5]); ss >> tx;          //cm
  ss.clear(); ss.str(argv[6]); ss >> ty;          //cm
  ss.clear(); ss.str(argv[7]); ss >> tz;          //cm
  ss.clear(); ss.str(argv[8]); ss >> focalLength; //cm
  ss.clear(); ss.str(argv[9]); ss >> ratio;       //ratio pixel/cm

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

  // Compute the camera projection matrix (world 3D points to camera 2D points)
  Eigen::Matrix4d inverseCameraPoseMatrix = Eigen::Matrix4d::Identity();
  for(unsigned int i=0;i<4;i++)
  {
    for(unsigned int j=0;j<4;j++)
    {
	inverseCameraPoseMatrix(i,j)=cameraPoseMatrix[i*4+j];
    }
  }
  inverseCameraPoseMatrix = inverseCameraPoseMatrix.inverse().eval();
  Eigen::Matrix<double,3,4> extrinsicCameraMatrix = inverseCameraPoseMatrix.block(0,0,3,4);
  Eigen::Matrix3d intrinsicCameraMatrix = Eigen::Matrix3d::Identity();
  intrinsicCameraMatrix(0,0)=intrinsicCameraMatrix(1,1)=focalLength*ratio;
  Eigen::Matrix<double,3,4> projectionMatrix = intrinsicCameraMatrix*extrinsicCameraMatrix;

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

  // Print and display the 3D points and their projections on the camera image plane
  unsigned int numPoints = 1;
  for(unsigned int pi = 0; pi < numPoints ; pi++)
  {
    // Print out the 3D coordinates of the specified points (by their index)
    int pIdx = 150;
    double p[3];
    polydata->GetPoint(pIdx,p);
    std::cout << "3D coordinates " << pIdx << " : (" << p[0] << " " << p[1] << " " << p[2] << ")" << std::endl;
    
    // Display the selected point in 3D
    display3DPointAndIndex(renderer,p[0],p[1],p[2],pIdx);
  
    // Print out the projections of the selected 3D points
    Eigen::Vector4d h3Dpoint; h3Dpoint(0)=p[0]; h3Dpoint(1)=p[1]; h3Dpoint(2)=p[2]; h3Dpoint(3)=1.0;
    Eigen::Vector3d h2Dpoint = projectionMatrix*h3Dpoint;
    std::cout<<"2D projection " << pIdx << " : (" << h2Dpoint(0)/h2Dpoint(2) 
                                           << " " << h2Dpoint(1)/h2Dpoint(2) << ")" << std::endl;
  }
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}
