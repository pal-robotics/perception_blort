 /**
 * @file main.cpp
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file for standalone version of TomGine rendering engine.
 */
 
#include <stdio.h>

#include <blort/TomGine/tgEngine.h>
#include <blort/TomGine/tgRenderModel.h>
#include <blort/TomGine/tgModelLoader.h>
#include <blort/TomGine/tgShapeCreator.h>
#include <blort/TomGine/tgCollission.h>
#include <blort/TomGine/tgFont.h>

#include <time.h>

using namespace TomGine;
using namespace std;

typedef vector<vec3> PointList;

// float avgI16FB(int x, int y, unsigned res, tgTexture &tex){
// 	int lvl = ilog2(res)+1;
// 	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_INTENSITY16, x, y, res, res, 0);
// 	
// 	float pix[4] = {0,0,0,0};
// 	glGetTexImage(GL_TEXTURE_2D, lvl-1, GL_RED, GL_FLOAT, pix);
// 	float avg = 0;
// 	for(unsigned i=0; i<4; i++)
// 		avg += 0.25 * pix[i];
// 	return avg;
// }

int main(int argc, char *argv[])
{
	unsigned width = 800;
	unsigned height = 600;
//	char avg_time[128];
	tgEngine render(width,height, 1.0f, 0.1f, "TomGine Render Engine", true);
	
	printf("\n Demo TomGine\n\n");
	
	printf(" TomGine control\n");
	printf(" -------------------------------------------\n");
	printf(" [Left Mouse Button] Rotate\n");
	printf(" [Right Mouse Button] Move\n");
	printf(" [Scroll Wheel] Zoom\n");
	printf(" [w] Switch to wireframe mode\n");
	printf(" [f] Switch to flat/smooth shading\n");
	printf(" \n\n");
	
	srand(time(NULL));
	float fTime;
	tgTimer timer;
	PointList m_points;
	vec3 v;
	tgShapeCreator shape_creator;
	
	printf("[TomGine/main.cpp] Warning HARDCODED font file\n");
	
	// Load Model
	// for more materials visit: http://wiki.delphigl.com/index.php/Materialsammlung
	tgMaterial matSilver;
	matSilver.ambient = vec4(0.19f,0.19f,0.19f,1.0f);
	matSilver.diffuse = vec4(0.51f,0.51f,0.51f,1.0f);
	matSilver.specular = vec4(0.77f,0.77f,0.77f,1.0f);
	matSilver.shininess = 51.2f;
	
	tgMaterial matRed;
	matRed.Color(1.0f, 0.0f, 0.0f);
	
	tgMaterial matBlueBlend;
	matBlueBlend.Color(0.0f, 0.0f, 1.0f, 0.5f);
		
	tgRenderModel camera;
	tgRenderModel camera2;
	tgModelLoader loader;
	printf("[TomGine/main.cpp] Warning HARDCODED ply file\n");
	loader.LoadPly(camera, "../Resources/ply/camera.ply");
	loader.LoadPly(camera2, "../Resources/ply/camera.ply");
	
	camera.m_material = matRed;
	camera.m_pose.t = vec3(0.0f,0.1f,0.0f);
	camera.ComputeBoundingSphere();
	
	camera2.m_pose.t = vec3(0.1f,0.1f,0.0f);
	camera2.ComputeBoundingSphere();
	
	tgRenderModel shape;
	
// 	shape_creator.CreateSphere(shape, 0.05, 3, ICOSAHEDRON);
// 	shape_creator.CreateBox(shape, 0.1,0.1,0.1);
	shape_creator.CreateCylinder(shape, 0.1f, 0.2f, 64, 2, true);
	shape.m_material = matBlueBlend;
		
	tgRenderModel poly;
// 	std::vector<vec3> pointlist;
// 	pointlist.push_back(vec3(0.1f,0.01f,-0.1f));
// 	pointlist.push_back(vec3(0.00f,0.0,-0.03f));
// 	pointlist.push_back(vec3(-0.1f,0.01f,-0.1f));
// 	pointlist.push_back(vec3(0.0f,0.01f,0.2f));
// 	shape_creator.TriangulatePolygon(poly, pointlist);
// 	poly.ComputeFaceNormals();

	tgRay ray;
	ray.start = render.GetCameraPosition();
	ray.dir = vec3(0.01f,0.0f,0.0f) - (render.GetCameraPosition()*2.0f);
	
	vec3 p;
	std::vector<vec3> pl;
	std::vector<vec3> nl;
	std::vector<double> zl;
	
// 	bool b = tgCollission::IntersectRayTriangle(p, z, ray, pointlist[0], pointlist[1], pointlist[2]);
	bool b = tgCollission::IntersectRayModel(pl, nl, zl, ray, shape);
	
// 	tgTexture tex;
// 	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
// 	tex.Load("../Resources/img/smiley.jpg");

	//tgFont m_font;
	
	// Rendering loop
	while(render.Update(fTime=timer.Update())){
		
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
			glVertex3f(ray.start.x, ray.start.y, ray.start.z);
			glVertex3f(ray.start.x + ray.dir.x, ray.start.y + ray.dir.y, ray.start.z + ray.dir.z);
		glEnd();
		glDisable(GL_TEXTURE_2D);
		
		render.Activate3D();
		for(unsigned int i=0; i< pl.size(); i++)
		{
			glDisable(GL_DEPTH_TEST);
			glPointSize(3);
			glBegin(GL_POINTS);
				glColor3f(1.0f-b,b,0.0f);
				glVertex3f(pl[i].x, pl[i].y, pl[i].z);
			glEnd();
			glEnable(GL_DEPTH_TEST);
		}
		glEnable(GL_LIGHTING);
		
		//poly.DrawFaces();
		camera.m_pose.Rotate(0.0f,3.14f*fTime, 0.0f);
		glColor3f(1,1,1);
		if(tgCollission::IntersectModels(camera, camera2))
			render.PrintText2D("collission", vec2(50,50));
		
		camera.DrawFaces();
		camera2.DrawFaces();
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		matBlueBlend.Apply();
		camera.DrawBoundingSphere();
		camera2.DrawBoundingSphere();
		shape.DrawFaces();
		glDisable(GL_BLEND);
		
		glColor3f(1,1,1);
		render.PrintText3D("Logitech", camera.m_pose.t);
		
		// Super fast calculation of average
// 		render.Activate2D();
// 		timer.Update();
// 		glEnable(GL_TEXTURE_2D);
// 		tgTexture tex;
// 		for(unsigned i=0; i<100; i++){
// 			avgI16FB(x,y,res,tex);
// 		}
// 		sprintf(avg_time, "Averaging time: %.0f ms", timer.Update() * 1000);
// 		m_font.Print(avg_time, 18, width - 200, 7);	
 		
 		glColor3f(1,1,1);
 		render.PrintText2D("TomGine Render Engine", vec2(7,7), 12);
 		
#ifdef LINUX
			usleep(5000);		// not to overload GPU
#endif

#ifdef WIN32
			Sleep(10);
#endif

	}
	
	return 0;
}



