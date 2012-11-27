#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPLYReader.h>
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
#include <iostream>
#include <fstream>

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
  std::cout << std::endl << "----------------" << std::endl;
  std::cout << "VirtualCameraVTK" << std::endl;
  std::cout << "----------------" << std::endl;

  // Parse command line arguments
  if(argc != 13)
    {
    std::cout << "Usage: " << argv[0] << " <mesh.ply> (mesh file .ply) yaw pitch roll (camera rotation [rad]) tx ty tz (camera translation [cm]) f (focal length [cm] > 0) cx (X camera principal point [pixels]) cy (Y camera principal point [pixels]) r (world to image ratio [pixels/cm]) indexes (points indexes file name)" << std::endl;
    return EXIT_FAILURE;
    }

  // Read the mesh from (.ply) file
  std::string filename = argv[1];
  vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName (filename.c_str());
  reader->Update(); 

  // Read the filename for the point in file
  std::string pointIndexesFileName = argv[12];

  // Create a mapper and actor for the mesh
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Get the mesh data
  vtkPolyData* polydata = mapper->GetInput();
  unsigned int numVertices = polydata->GetNumberOfPoints();
  std::cout << "Number of vertices: " << numVertices << std::endl;

  // Read the extrinsic and intrinsic camera parameters
  std::istringstream ss;  
  double yaw,pitch,roll,tx,ty,tz,focalLength,cx,cy,ratio;
  ss.clear(); ss.str(argv[2]); ss >> yaw;         //rad
  ss.clear(); ss.str(argv[3]); ss >> pitch;       //rad
  ss.clear(); ss.str(argv[4]); ss >> roll;        //rad
  ss.clear(); ss.str(argv[5]); ss >> tx;          //cm
  ss.clear(); ss.str(argv[6]); ss >> ty;          //cm
  ss.clear(); ss.str(argv[7]); ss >> tz;          //cm
  ss.clear(); ss.str(argv[8]); ss >> focalLength; //cm
  ss.clear(); ss.str(argv[9]); ss >> cx;          //pixels
  ss.clear(); ss.str(argv[10]); ss >> cy;         //pixels
  ss.clear(); ss.str(argv[11]); ss >> ratio;       //ratio pixel/cm

  std::cout << "Camera extrinsic parameters:"<< std::endl;
  std::cout << "	- yaw   : " << yaw << " rad" << std::endl; 
  std::cout << "	- pitch : " << pitch << " rad" << std::endl; 
  std::cout << "	- roll  : " << roll << " rad" << std::endl; 
  std::cout << "	- tx    : " << tx << " cm" << std::endl; 
  std::cout << "	- ty    : " << ty << " cm" << std::endl; 
  std::cout << "	- tz    : " << tz << " cm" << std::endl; 
  std::cout << "Camera intrinsic parameters:" << std::endl;
  std::cout << "	- f	: " << focalLength << " cm" << std::endl;
  std::cout << "	- cx	: " << cx << " pixels" << std::endl;
  std::cout << "	- cy	: " << cy << " pixels" << std::endl;
  std::cout << "World to image ratio:" << std::endl;
  std::cout << "	- r	: " << ratio << " pixels/cm" << std::endl;

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
  double cos_y,cos_p,cos_r,sin_y,sin_p,sin_r;
  cos_y=cos(yaw); cos_p=cos(pitch); sin_y=sin(yaw);
  sin_p=sin(pitch);sin_r=sin(roll); cos_r=cos(roll);
  const double cameraPoseMatrix[]={cos_y*cos_p , cos_y*sin_p*sin_r-sin_y*cos_r , cos_y*sin_p*cos_r+sin_y*sin_r , tx ,
                                   sin_y*cos_p , sin_y*sin_p*sin_r+cos_y*cos_r , sin_y*sin_p*cos_r-cos_y*sin_r , ty ,
                                   -sin_p   , cos_p*sin_r          , cos_p*cos_r          , tz ,
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
  //std::ostream objOstream (cout.rdbuf());
  //camera->Print(objOstream);

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
  intrinsicCameraMatrix(0,2)=cx;
  intrinsicCameraMatrix(1,2)=cy;
  Eigen::Matrix<double,3,4> projectionMatrix = intrinsicCameraMatrix*extrinsicCameraMatrix;

  // Add the world axes
  vtkSmartPointer<vtkAxesActor> axes =
    vtkSmartPointer<vtkAxesActor>::New();
  axes->SetTotalLength(10.0,10.0,10.0); 	

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

  // Read the indexes of a subset of the mesh 3D points. Print and display the selected 3D points
  // and their projections on the camera image plane. 
  std::string line;
  std::ifstream pointIndexesFile (pointIndexesFileName.c_str());
  if (pointIndexesFile.is_open())
  {
    while ( pointIndexesFile.good())
    {
      // Read the next point index from file
      getline (pointIndexesFile,line);
      std::istringstream buffer(line); unsigned int pIdx; buffer >> pIdx; buffer.clear(); 
      if(pointIndexesFile.eof()){break;}

      if(pIdx<0 || pIdx>=numVertices)
      {
        std::cout<<"Invalid point index" << std::endl;
        return EXIT_FAILURE;
      }

      // Get the 3D coordinates of the selected point
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
    pointIndexesFile.close();
  }
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}
