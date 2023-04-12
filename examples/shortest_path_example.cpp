#include "vtkAutoInit.h" 
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkVertex.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkProperty.h>
#include <vtkNamedColors.h>
#include <vtkStringArray.h>
#include <vtkPointData.h>
#include <vtkTextProperty.h>
#include <vtkPointSetToLabelHierarchy.h>
#include <vtkLabelPlacementMapper.h>
#include <vtkActor2D.h>
#include <vtkInteractorStyleTrackballCamera.h>


#include <ctime>
#include <fstream>
#include <exception> 
#include "shortest_path_wrapper.h"

//声明
vtkSmartPointer<vtkPolyData> vtkpointdata(std::vector<int> path, vtkSmartPointer<vtkPLYReader> reader);
vtkSmartPointer<vtkActor> pointactor(vtkSmartPointer<vtkPolyData> polydata, float color[3], int size);

using namespace ortho;
using Mesh = pmp::SurfaceMesh;
using Point = pmp::Point;
using Vertex = pmp::Vertex;


//-------- short path example ----------------
int main_1() {
	clock_t startTime, endTime;
	pmp::SurfaceMesh mesh_ply;
	startTime = clock();
	std::string filename = "E:/data_jyx_line/upper_special.ply";
	//mesh_ply.read("E:\\hehz_short_path\\view.ply");
	//mesh_ply.read("E:/data_jyx_line/upper_with_jaw.ply");
	//int v_start = 48706, v_end = 54007;
	//mesh_ply.read("E:/data_jyx_line/upper_special_part.ply");
	//int v_start = 3789, v_end = 1370;   //(1217, 1370) (1370,3739) ( 3739,1173) (3739,1861) 
	mesh_ply.read(filename);
	//int v_start = 8833, v_end = 43431;   //special:(8833, 43431) 凸起 （68211，51261） 正常：（32001，32268）
	std::cout << "----------begin----------" << std::endl;
	startTime = clock();
	ortho::ShortestPathClass sp(mesh_ply); 
	//std::vector<int> keyid = { 135035 ,150727 ,130843 ,130822 ,122337 ,126626 ,211160,233072};
	std::vector<int> keyid = { 105281 ,111419 ,122326 ,150727 ,150727 ,130731,122326,135147 };
	int v_start = keyid[0], v_end = keyid[1];   //边界与牙龈萎缩：128593 148425  //孔洞处：120165 146231 130784  / 130784(118079/122326) 150727 130731(130683)
	//int v_start = 128593, v_end = 128616;       //正常： 105281 111419 /142706,151752 /    牙龈萎缩：128593, 148425 
	std::vector<int> ret = sp.getShortestPath(v_start, v_end);

	v_start = keyid[2], v_end = keyid[3];  //牙龈萎缩上方：146231 135035 132831(139350)   浅牙龈萎缩： 122337 126626 / 上方：130843 130822
	std::vector<int> ret1 = sp.getShortestPath(v_start, v_end);

	v_start = keyid[4], v_end = keyid[5];
	std::vector<int> ret2 = sp.getShortestPath(v_start, v_end);

	v_start = keyid[6], v_end = keyid[7];
	std::vector<int> ret3 = sp.getShortestPath(v_start, v_end);

	std::cout << std::endl << (double)(clock() - startTime) << std::endl;

	// 
	/*
	startTime = clock();
	for (int i = 0; i < 10; i++) {
		startTime = clock();
		ret = sp.getShortestPath(v_start, v_end);
		std::cout << (double)(clock() - startTime) << " ";
	}
	*/

	//------------------------------------------------------
	//vtk visulization  --   shortest path
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName("E:/data_jyx_line/upper_special.ply");
	reader->Update();
	std::cout << reader->GetOutput()->GetNumberOfCells() << ' ' << reader->GetOutput()->GetNumberOfPoints() << std::endl;

	//point struction
	float color1[3] = { 1,0,0 }, color2[3] = { 0,1,0 }, color3[3] = { 0,0,1 };
	int size1 = 6, size2 = 6;
	vtkSmartPointer<vtkPolyData> polydata = vtkpointdata(ret, reader);
	auto pointActor = pointactor(polydata, color1, size1);

	vtkSmartPointer<vtkPolyData> polydata1 = vtkpointdata(ret1, reader);
	auto pointActor1 = pointactor(polydata1, color2, size2);

	vtkSmartPointer<vtkPolyData> polydata2 = vtkpointdata(ret2, reader);
	auto pointActor2 = pointactor(polydata2, color3, size2);

	vtkSmartPointer<vtkPolyData> polydata3 = vtkpointdata(ret3, reader);
	auto pointActor3 = pointactor(polydata3, color1, size2);

	vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	meshMapper->SetInputData(reader->GetOutput());
	vtkSmartPointer<vtkActor> meshActor = vtkSmartPointer<vtkActor>::New();
	meshActor->GetProperty()->SetColor(1, 1, 1);
	meshActor->SetMapper(meshMapper);

	vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetWindowName("Axes");
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1200, 900);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkNew<vtkInteractorStyleTrackballCamera> style;
	renderWindowInteractor->SetInteractorStyle(style);

	renderer->AddActor(meshActor);
	renderer->AddActor(pointActor);
	renderer->AddActor(pointActor1);
	renderer->AddActor(pointActor2);
	renderer->AddActor(pointActor3);

	renderer->SetBackground(colors->GetColor3d("Mint").GetData());
	renderWindow->Render();
	renderWindowInteractor->Initialize();
	renderWindowInteractor->Start();

	return 0;
}


//Function
vtkSmartPointer<vtkPolyData> vtkpointdata(std::vector<int> path, vtkSmartPointer<vtkPLYReader> reader) {
	vtkSmartPointer<vtkPoints> ps = vtkSmartPointer<vtkPoints>::New();
	for (int i : path)
	{
		auto point = reader->GetOutput()->GetPoint(i);
		ps->InsertNextPoint(point[0], point[1], point[2]);
	}

	vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
	vertex->GetPointIds()->SetNumberOfIds(ps->GetNumberOfPoints());
	for (int i = 0; i < ps->GetNumberOfPoints(); i++) {
		vertex->GetPointIds()->SetId(i, i);
	}

	vtkSmartPointer<vtkCellArray> vertices =
		vtkSmartPointer<vtkCellArray>::New();
	vertices->InsertNextCell(vertex);

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(ps);
	polydata->SetVerts(vertices);
	return polydata;
}

vtkSmartPointer<vtkActor> pointactor(vtkSmartPointer<vtkPolyData> polydata, float color[3], int size) {
	vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pointMapper->SetInputData(polydata);
	vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
	Actor->GetProperty()->SetColor(color[0], color[1], color[2]);
	Actor->GetProperty()->SetPointSize(size);
	Actor->SetMapper(pointMapper);
	return Actor;
}




int main() {
	clock_t startTime, endTime;
	pmp::SurfaceMesh mesh_ply;
	startTime = clock();
	std::string filename = "F:/TE_error_data/lower_1024.ply";
	//mesh_ply.read("E:\\hehz_short_path\\view.ply");
	//mesh_ply.read("E:/data_jyx_line/upper_with_jaw.ply");
	//int v_start = 48706, v_end = 54007;
	//mesh_ply.read("E:/data_jyx_line/upper_special_part.ply");
	//int v_start = 3789, v_end = 1370;   //(1217, 1370) (1370,3739) ( 3739,1173) (3739,1861) 
	mesh_ply.read(filename);
	//int v_start = 8833, v_end = 43431;   //special:(8833, 43431) 凸起 （68211，51261） 正常：（32001，32268）
	std::cout << "----------begin----------" << std::endl;
	startTime = clock();
	ortho::ShortestPathClass sp(mesh_ply,false);

	std::ofstream outfile("F:/TE_error_data/lower_1024_path_noweight.json");
	outfile << "{ \"L_43\":[";
	startTime = clock();
	std::vector<int> keyid = { 99876,61330,48889,50911,57032,91116,119286,141869,141993,121066,107582,98534,83373,89816,116476,128385,136846,106921,99876 };
	//std::vector<int> keyid = { 99876,61330,48889,50911,57032,91116,119286,141869,141993,121066,107582,102858,98509, 116476,128385,136846,106921,99876 };

	for (int i = 0; i < keyid.size()-1; i++) {
		startTime = clock();
		std::vector<int> ret = sp.getShortestPath(keyid[i], keyid[i+1]);
		std::cout << (double)(clock() - startTime) << " ";

		for (int j = 0; j < ret.size(); ++j)
			outfile << ret[j] << ", ";
	}
	outfile << keyid[keyid.size()-1] << "]}";
	outfile.close();

	
	return 0;
}

