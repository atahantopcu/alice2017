#define _MAIN_
#define _ALG_LIB_



#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"
#include "largeMesh.h"

////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------




graphStack GS;
//Graph G;
//Mesh M;

vec minV, maxV;
double iter;
int currentPointId;

bool run = false;
int rCnt = 0;
////// --- GUI OBJECTS ----------------------------------------------------

//SliderGroup S;
//ButtonGroup B;
bool showRobot = false;
bool showGraphStackData = false;
bool showGraphStackMesh = false;
bool showMeshWire = false;

char s[200],text[200], text1[200], jts[400];

double lightscale = 3.0;
double background = 0.8;
double dummy = 0;

double resampleDist = 0.2;
double rotateBy = 0;
double translateBy = 1 ;
double axis = 0;
bool flipNormals = false;
vec camPt;
////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

//largeMesh LM;
int fileNum = 0;
string printfile;
////// ---------------------------------------------------- MODEL  ----------------------------------------------------

void setup()
{



	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	//////////////////////////////////////////////////////////////////////////

	GS = *new graphStack();
	GS.readGraphAndCreateDataMesh("data/tree_pts.txt", 1.0);//circular_pts

	//////////////////////////////////////////////////////////


	S = *new SliderGroup();


	GS.threshold = 0;
	S.addSlider(&GS.threshold, "threshold");
	S.sliders[0].minVal = 0;
	S.sliders[0].maxVal = 50;
	S.addSlider(&resampleDist, "resampleDist");

	S.addSlider();//dumy
	S.addSlider();//dumy
	S.addSlider();//dumy


	S.addSlider(&rotateBy, "rotateBy");
	S.sliders[S.numSliders - 1].minVal = -90;
	S.sliders[S.numSliders - 1].maxVal = 90;

	S.addSlider(&axis, "axis");
	S.sliders[S.numSliders - 1].maxVal = 3;

	S.addSlider(&translateBy, "translateBy");
	S.sliders[S.numSliders - 1].maxVal = 5;


	S.addSlider();//dumy
	S.addSlider();//dumy
	S.addSlider();//dumy
	
	S.addSlider(&lightscale, "lightscale");
	S.sliders[S.numSliders - 1].maxVal = 40;
	S.addSlider(&background, "background");
	/////////////////////////////

	B = *new ButtonGroup(vec(50, 650, 0));
	B.addButton(&showGraphStackData, "showGraphData");
	B.addButton(&showGraphStackMesh, "showStackMesh");
	B.addButton(&showMeshWire, "showMeshWire");
	B.addButton(&flipNormals, "flipNormals");

	//////////////////////////////////////////////////////////////////////////

	
}

void update(int value)
{

	if (run)for (int i = 0; i < 5; i++)keyPress(' ', 0, 0);

}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(background);
	drawGrid(20.0);



	S.draw();
	B.draw();
	// ------------------------ draw the path points / Tool orientations 

	GS.draw(showGraphStackMesh, showMeshWire,showGraphStackData);

	//////////////////////////////////////////////////////////


	glColor3f(0, 0, 0);
	setup2d();

	AL_drawString(s, winW * 0.5, winH - 50);
	AL_drawString(text, winW * 0.5, winH - 75);
	AL_drawString(jts, winW * 0.5, winH - 100);
	AL_drawString(printfile.c_str(), winW * 0.5, winH - 125);
	
	

	int hts = 50;
	int wid = winW * 0.75;

	AL_drawString("  c : convert contour to cyclic countour;", wid, hts); hts += 25;;
	AL_drawString("  SPACE : smooth current contour", wid, hts); hts += 25;
	AL_drawString("  U : smoothIteration toggle;", wid, hts); hts += 25;
	AL_drawString("  n :  evenly distribute points of contour ;", wid, hts); hts += 25;
	AL_drawString("  - :  reduce points of contour;", wid, hts); hts += 25;
	AL_drawString("  i :  inflate vertices of contour", wid, hts); hts += 25;
	AL_drawString("  i :  inflate vertices of contour w.r.t graph (red)", wid, hts); hts += 25;

	AL_drawString("  b : add current contour to print stack", wid, hts); hts += 25;
	AL_drawString("  r : rotate & translate contour ", wid, hts); hts += 25;
	
	AL_drawString("  B : multi-transform preset _01 ", wid, hts); hts += 25;

	AL_drawString("  W : write stack to file; ", wid, hts); hts += 25;
	AL_drawString("  R : reset ", wid, hts); hts += 25;
	hts += 25;

	
	restore3d();

	drawVector(camPt, vec(wid, hts + 25, 0), "cam");

}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{

	///// GRAPH GENERTOR PROGRAM 

	
	////////////////////////////////////////////////////////////////////////// -------------- 


	if (k == 'c')GS.convertContourToToroidalGraph();
	if (k == ' ')GS.smoothCurrentGraph(180);
	if (k == 'n')
	{
		GS.MM.G.redistribute_toroidal(resampleDist);
		for (int i = 0; i < 10; i++)GS.MM.G.smoothGraph(90);
	}

	if (k == '-')GS.reducePointsOnContourGraph(2);

	if (k == 'i')GS.inflateCurrentGraph();
	if (k == 'I')GS.inflateCurrentGraph(false);

	if (k == 'b')
	{
		GS.addCurrentContourGraphToPrintStack(translateBy, 0.0);
		GS.transformCurrentGraphBy(int(rotateBy), int(axis), vec(0, 0, translateBy));
		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}

	if (k == 'r')
	{
		GS.transformCurrentGraphBy(int(rotateBy), int(axis), vec(0, 0, translateBy) );
		GS.addCurrentContourGraphToPrintStack(translateBy, 0.0);
		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}

	if (k == '<')GS.popPrintstack();

	if (k == 'O')
	{
		showGraphStackMesh = false;
		GS.convertedToToroidal = false;
		GS.currentStackLayer = 0;
		GS.LM.n_v = GS.LM.n_f = 0;
	}


	//

	if (k == 'B')
	{
		for (int i = 0; i < 10; i++)
		{
			keyPress('b', 0, 0);
			
			for (int j = 0; j < 1; j++)GS.inflateCurrentGraph();
			for (int j = 0; j < 30; j++)keyPress(' ', 0, 0);
			for (int j = 0; j < 1; j++)GS.MM.G.inflateVertices();
		}

		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}

	


	if (k == 'Q')GS.writeCurrentGraph();
	if (k == 'W')
	{
		GS.writeStackToFile("data/PRINT.txt");
		GS.writeStackToObj("data/PRINT.obj");
		GS.LM.writeOBJ("data/largeMesh.obj");
	}

	if (k == 'e')GS.readGraphAndCreateDataMesh("data/tree_pts.txt", 1.0);//circular_pts

	if (k == 'U')run = !run;



}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		
		B.performSelection(x, y);
		S.performSelection(x, y, HUDSelectOn);

		if (HUDSelectOn & !GS.convertedToToroidal)
		{
			//GS.threshold = 1.5;
			GS.createIsoContourGraph(GS.threshold);
			cout << GS.MM.G.n_v << endl;

		}
	}

	if((GLUT_LEFT_BUTTON == b && GLUT_UP == state) || (GLUT_RIGHT_BUTTON == b && GLUT_UP == state))
	{
		int cur_msx = winW * 0.5;
		int cur_msy = winH * 0.5;
		camPt = screenToCamera(cur_msx, cur_msy, 0.2);

		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
	if (HUDSelectOn & !GS.convertedToToroidal)
	{
		GS.createIsoContourGraph(GS.threshold);
		//GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}

	bool dragging = (glutGetModifiers() == GLUT_ACTIVE_ALT) ? true : false;
	int cur_msx = winW * 0.5;
	int cur_msy = winH * 0.5;
	camPt = screenToCamera(cur_msx, cur_msy, 0.2);

	//if( dragging)GS.LM.updateColorArray(lightscale, flipNormals, camPt);

}




#endif // _MAIN_
