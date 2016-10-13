#include <windows.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include "scene_io.h"
#include "Timer.h"
#include "CImg.h"
#include "Eigen\Eigen"
#include "Eigen\Dense" 
#include <vector>
#include <assert.h>
#include <random>
#include "pfmLoader.h"

#define PI 3.14159265
#define MOVE_FACTOR 0.001
#define FLT_MAX         3.402823466e+38F 
#define MAX_DEPTH 3
#define PER_PIXAL 2
#define MAX_SAMPLE 128
#define DISTANCE 80
#define SHOT_TIME 0.08
#define INTEGRATE 4
#define FOG 3.5

const float sigma_s = 0.009, sigma_a = 0.015, sigma_t = sigma_s+sigma_a;


using namespace cimg_library;
using namespace Eigen;
using Eigen::MatrixXd;

float RaySphereIntersection (Vector3f SphereCenter, float Radius, Vector3f RayDir, Vector3f RayOri);

float RandomFloat();
Vector3f getVertical(Vector3f A);

Vector3f RayTriangleIntersection (Vector3f V0, Vector3f V1, Vector3f V2, Vector3f RayDir, Vector3f RayOri);
Vector3f getRefractionDir (Vector3f Normal, Vector3f RayDir, float n1, float n2 );
class BBox;
boolean Ray_Box_Interset(BBox box, Vector3f RayDir, Vector3f RayOri);
Vector3f randomDir();
Vector3f randomAllDir();
Vector3f specularPathTraceDir(Vector3f RView);

float clamp(float x);
#define IMAGE_WIDTH		600
#define IMAGE_HEIGHT	600

CImg<unsigned char>  theImage(IMAGE_WIDTH,IMAGE_HEIGHT,1,3,0);
enum PolyType {Sphere, Triangle};
typedef unsigned char u08;

//Global variables
class Previews;
SceneIO *scene = NULL;

HDRImage *hdrImage = new HDRImage; 

std::vector<Previews>  previews;
int NumberOfPrimes=0;
int totalBVH_TreeNode=0;


Vector3f E;
Vector3f V;
Vector3f U;
Vector3f A;
float phi;
Vector3f B;
float c = 1.5;
int TrT = 5;

Vector3f M;

Vector3f Y;
Vector3f X;

int traceTime = 0;
Vector3f Refraction(float ktran, Vector3f RefractionDir, Vector3f Position);
Vector3f Normalize_color(float x, float y, float z);
static Vector3f Luna(0.212671f, 0.715160f, 0.072169f);

class Ray
{
public:
	Vector3f Ori,Dir,Destination,Illumination;



	float distance;

	Ray()
	{
		Vector3f zero(0,0,0);
		Ori = Dir = Destination = Illumination = zero;
		distance = NULL;
	}

};

Vector3f AbsorbedIllumination(Ray ray);
Vector3f ViewResult(Ray ray, Vector3f LightSoursePosition, Vector3f LightColor);

class TraceResult 
{
public:
	bool IntersectOrNot;
	MaterialIO* Material;
	Vector3f Normal;
	float Min;
	Vector3f Position;

	TraceResult()
	{
		IntersectOrNot = 0;
		Min = INFINITE;
	}

	~TraceResult()
	{}
} ;

class BBox
{
public:              // Why some element must be define as private or protect? why not all public???????????????????????????????
	Vector3f pMin;
	Vector3f pMax;
	Vector3f centerPoint;

	BBox ()
	{
		Vector3f Infinity(FLT_MAX,FLT_MAX,FLT_MAX);
		Vector3f _Infinity(-FLT_MAX,-FLT_MAX,-FLT_MAX);
		Vector3f zero(0,0,0);
		pMin = Infinity;
		pMax = _Infinity;
		centerPoint = zero;
	}


	int MaximumExtent()  
	{
        Vector3f diag = pMax - pMin;
        if (diag[0] > diag[1] && diag[0] > diag[2])
            return 0;
        else if (diag[1] > diag[2])
            return 1;
        else
            return 2;
    }

	float SurfaceArea() 
	{
        Vector3f d = pMax - pMin;
        return 2.f * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
    }
};









class Previews
{
public:
	//Variables both for Triangle and Sphere
	
	BBox Box;
	enum PolyType type;

	//Variable for Triangle
	struct VertexIO *vert;
	NormType normType;
	 MaterialBinding materialBinding;

	//Variable for Sphere
	Vector3f SphereOri;
	float radius;
	MaterialIO *material;
};



BBox Union(const BBox b, Vector3f p) 
{
    
	Vector3f result_max;
	Vector3f result_min;

    result_max[0] = max(b.pMax[0],p[0]);
	result_max[1] = max(b.pMax[1],p[1]);
	result_max[2] = max(b.pMax[2],p[2]);

	result_min[0] = min(b.pMin[0],p[0]);
	result_min[1] = min(b.pMin[1],p[1]);
	result_min[2] = min(b.pMin[2],p[2]);

	BBox result;
	result.pMax = result_max;
	result.pMin = result_min;
	result.centerPoint = (result_max + result_min) / 2;
	return result;
}


BBox Union(BBox x, BBox y)
{
	Vector3f result_max;
	Vector3f result_min;

	result_max[0] = max(x.pMax[0],y.pMax[0]);
	result_max[1] = max(x.pMax[1],y.pMax[1]);
	result_max[2] = max(x.pMax[2],y.pMax[2]);

	result_min[0] = min(x.pMin[0],y.pMin[0]);
	result_min[1] = min(x.pMin[1],y.pMin[1]);
	result_min[2] = min(x.pMin[2],y.pMin[2]);

	BBox result;
	result.pMax = result_max;
	result.pMin = result_min;
	result.centerPoint = (result_max + result_min) / 2;
	return result;
}








static void loadScene2Previews(SceneIO *scene)
{	
	ObjIO * FirstObj = scene->objects;

	while(scene->objects != NULL)
	{
		if(scene->objects->type == SPHERE_OBJ) //Sphere
		{
			Previews thisPreview;

			NumberOfPrimes++;

			thisPreview.type = Sphere;
			SphereIO* sphere = NULL;
			sphere = (SphereIO*) (scene->objects->data);
			Vector3f SphereCenter;
			float Radius;
			SphereCenter[0] = sphere->origin[0];
			SphereCenter[1] = sphere->origin[1];
			SphereCenter[2] = sphere->origin[2];

			thisPreview.radius = sphere->radius;
			thisPreview.material = scene->objects->material;
			thisPreview.SphereOri = SphereCenter;

			Radius = sphere->radius;

			Vector3f VecRadius(Radius,Radius,Radius);
			thisPreview.Box.pMax = SphereCenter + VecRadius;
			thisPreview.Box.pMin = SphereCenter - VecRadius;
			thisPreview.Box.centerPoint = SphereCenter;

			previews.push_back(thisPreview);
		}
				
		else        //PolySet
		{
			PolySetIO* polySet = NULL;
			polySet = (PolySetIO*) (scene->objects->data);

			
			for(int i = 0; i < polySet->numPolys; i++)
			{
				Previews thisPreview;

				NumberOfPrimes++;

				thisPreview.materialBinding = polySet->materialBinding;

				thisPreview.material = scene->objects->material;

				thisPreview.type = Triangle;
				PolygonIO poly;
				poly = polySet->poly[i];  
				thisPreview.vert = poly.vert;

				Vector3f V0,V1,V2;
				VertexIO vertex;
				vertex = poly.vert[0];
				V0[0] = vertex.pos[0];
				V0[1] = vertex.pos[1];
				V0[2] = vertex.pos[2];
				

				vertex = poly.vert[1];
				V1[0] = vertex.pos[0];
				V1[1] = vertex.pos[1];
				V1[2] = vertex.pos[2];
				

				vertex = poly.vert[2];
				V2[0] = vertex.pos[0];
				V2[1] = vertex.pos[1];
				V2[2] = vertex.pos[2];

				if (polySet->normType == PER_VERTEX_NORMAL)
				{thisPreview.normType = PER_VERTEX_NORMAL;}
				if(polySet->normType == PER_FACE_NORMAL)
				{thisPreview.normType = PER_FACE_NORMAL;}

				// Give the BBox information of thisPrimitive
				Vector3f thisMax,thisMin;
				thisMax[0] = max(max(V0[0],V1[0]),V2[0]);
				thisMax[1] = max(max(V0[1],V1[1]),V2[1]);
				thisMax[2] = max(max(V0[2],V1[2]),V2[2]);
				thisMin[0] = min(min(V0[0],V1[0]),V2[0]);
				thisMin[1] = min(min(V0[1],V1[1]),V2[1]);
				thisMin[2] = min(min(V0[2],V1[2]),V2[2]);

				thisPreview.Box.pMax = thisMax;
				thisPreview.Box.pMin = thisMin;
				thisPreview.Box.centerPoint = (thisMax + thisMin)/2;
				previews.push_back(thisPreview);
			}
		}
		scene->objects = scene->objects->next;
	}
	scene->objects = FirstObj; 
}	



struct LinearBVHNode {
    BBox bounds;
    union {
		int primitivesOffset;    // leaf
        int secondChildOffset;   // interior
    };

    int nPrimitives;  // 0 -> interior node
    int axis;         // interior node: xyz
    int pad[2];       // ensure 32 byte total size
};

LinearBVHNode *nodes;



// Here, I will only save the Number of Position of the Primitive in the primitives Vector!!!!!!!!!!!!!!! Is that OK??????????????????????????????????
class BVH_TreeNode
{
public:
	 // BVHBuildNode Public Methods
	BVH_TreeNode() 
	{ 
		children[0] = children[1] = NULL;
		firstPrimOffset = NULL;
	}

	void InitLeaf(int first, int n, BBox &b) {
		firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
    }

	void InitInterior(int XYZ_012, BVH_TreeNode *left, BVH_TreeNode *right) {
        children[0] = left;
        children[1] = right;
        bounds = Union(left->bounds, right->bounds);
        splitAxis = XYZ_012;
        nPrimitives = 0;
    }

	BBox bounds;
	BVH_TreeNode *children[2];
    int splitAxis, nPrimitives,firstPrimOffset;   // 0 == X axis, 1 == Y axis, 2 == Z axis
	//Previews * leaf ;    //  ???????????????????????????????????????????????????????????? Here, Can I just set up a pointer to represent a Array???????????????????????????????????
};



struct CompareToBucket {
    CompareToBucket(int split, int num, int d, const BBox &b)
        : centroidBounds(b)
    { splitBucket = split; nBuckets = num; dim = d; }
	bool operator()(const Previews &p) const;

    int splitBucket, nBuckets, dim;
    const BBox &centroidBounds;
};


bool CompareToBucket::operator()(const Previews &p) const {
	int b = nBuckets * ((p.Box.centerPoint[dim] - centroidBounds.pMin[dim]) /
            (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
    if (b == nBuckets) b = nBuckets-1;
   // Assert(b >= 0 && b < nBuckets);/////////////////////////////////////////////////////////////////////////////////////////////
    return b <= splitBucket;
}

//
//
//int SAH_Calculater(int axis, int start, int end, BBox theOriBox)
//{
//	// Allocate _BucketInfo_ for SAH partition buckets
//	int mid;
//    const int nBuckets = 12;
//    struct BucketInfo 
//	{
//     BucketInfo() { count = 0; }
//     int count;
//     BBox bounds;
//	};
//
//     BucketInfo buckets[nBuckets];
//
//     
//	 
//	 // Initialize _BucketInfo_ for SAH partition buckets
//	 // What this Bucket used for????????????????????????????????????????????????????????????????????????????????????????????????????????
//     for (int i = start; i < end; ++i) 
//	 {
//		 int b = nBuckets *((previews[i].Box.centerPoint[axis] - theOriBox.pMin[axis]) /(theOriBox.pMax[axis] - theOriBox.pMin[axis]));
//        if (b == nBuckets) b = nBuckets-1;
//       // Assert(b >= 0 && b < nBuckets);
//        buckets[b].count++;
//		buckets[b].bounds = Union(buckets[b].bounds, previews[i].Box);
//     }
//	 // The above code I do not quite understand!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!??????????????????????????????????????????????????????????
//
//     // Compute costs for splitting after each bucket
//     float cost[nBuckets-1];
//     for (int i = 0; i < nBuckets-1; ++i) 
//	 {
//		 BBox b0, b1;
//		 int count0 = 0, count1 = 0;
//		 for (int j = 0; j <= i; ++j) 
//		 {
//			 b0 = Union(b0, buckets[j].bounds);
//			 count0 += buckets[j].count;
//		 }
//		 for (int j = i+1; j < nBuckets; ++j) 
//		 {
//			 b1 = Union(b1, buckets[j].bounds);
//			 count1 += buckets[j].count;
//		 }
//		 cost[i] = .125f + (count0*b0.SurfaceArea() + count1*b1.SurfaceArea()) / theOriBox.SurfaceArea();
//     }
//
//     // Find bucket to split at that minimizes SAH metric
//     float minCost = cost[0];
//     int minCostSplit = 0;
//     for (int i = 1; i < nBuckets-1; ++i) 
//	 {
//		if (cost[i] < minCost) 
//		{
//			minCost = cost[i];
//			minCostSplit = i;
//        }
//     }
//
//	 Previews *pmid = std::partition(&previews[start],
//                        &previews[end-1]+1,
//						CompareToBucket(minCostSplit, nBuckets, axis, centroidBounds));
//                    mid = pmid - &previews[0];
//
//}



BVH_TreeNode * recursiveBuild( int PreviewsArray_start, int PreviewsArray_end) 
{
   // assert(PreviewsArray_start != PreviewsArray_end);///////////////////////////////////////////////////////////////////////////////////////
	
	BVH_TreeNode * node	= new BVH_TreeNode;

	BBox current_box;
    for (int i = PreviewsArray_start; i < PreviewsArray_end; ++i)
	{
		current_box = Union(current_box, previews[i].Box);	// Compute bounds of all primitives in BVH node
	}	
	
	int nPrimitives = PreviewsArray_end - PreviewsArray_start;



	int mid;
	
    if (nPrimitives <= 10)								// Create leaf _BVHBuildNode_
	{        
		totalBVH_TreeNode++;
		if (nPrimitives == 0)
			int ddddkd = 0;
		node->InitLeaf(PreviewsArray_start, nPrimitives, current_box);    
		return node;
    }


    else   // have more than 5 primitives
	{

        // Compute bound of primitive centroids
		// First choose split Axis
        BBox centroidBounds;
        for (int i = PreviewsArray_start; i < PreviewsArray_end; ++i)
		{
			centroidBounds = Union(centroidBounds, previews[i].Box.centerPoint);  // Here, how to write the rewrite function?????????????????????????????????
		}
		int axis;
        axis = centroidBounds.MaximumExtent();

		


		if (centroidBounds.pMax[axis] == centroidBounds.pMin[axis]) 
		{
                // else if nPrimitives is greater than maxPrimsInNode, we
                // need to split it further to guarantee each node contains
                // no more than maxPrimsInNode primitives.
			mid = (PreviewsArray_start + PreviewsArray_end) / 2;
			node->InitInterior(axis,recursiveBuild(PreviewsArray_start, mid), recursiveBuild(mid, PreviewsArray_end));
			totalBVH_TreeNode++;
            return node;
            
        }



















		// calculate the cost, and find the place to parse!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!???????????????????????????????????????????
		// Allocate _BucketInfo_ for SAH partition buckets
		
		const int nBuckets = 12;
		struct BucketInfo 
		{
		 BucketInfo() { count = 0; }
		 int count;
		 BBox bounds;
		};

		 BucketInfo buckets[nBuckets];

     
	 
	 // Initialize _BucketInfo_ for SAH partition buckets

		 for (int i = PreviewsArray_start; i < PreviewsArray_end; ++i) // why here the i change to minus alot?????????????????????????????????????????????????
		 {
			//int temp = i;
			// Previews temp = previews[i];
			 float a1 = (previews[i].Box.centerPoint[axis] - centroidBounds.pMin[axis]);
			 float a2 = (centroidBounds.pMax[axis] - centroidBounds.pMin[axis]);
			 
			int b = nBuckets *(a1 /a2);



			if (b == nBuckets) b = nBuckets-1;
//			assert(b >= 0 && b < nBuckets);
			buckets[b].count++;
			buckets[b].bounds = Union(buckets[b].bounds, previews[i].Box);
			//i = temp;
		 }


		 // Compute costs for splitting after each bucket
		 float cost[nBuckets-1];
		 for (int i = 0; i < nBuckets-1; ++i) 
		 {
			 BBox b0, b1;
			 int count0 = 0, count1 = 0;
			 for (int j = 0; j <= i; ++j) 
			 {
				 b0 = Union(b0, buckets[j].bounds);
				 count0 += buckets[j].count;
			 }
			 for (int j = i+1; j < nBuckets; ++j) 
			 {
				 b1 = Union(b1, buckets[j].bounds);
				 count1 += buckets[j].count;
			 }
			 
			 
			 float cur1 = (count0*b0.SurfaceArea() + count1*b1.SurfaceArea());
			 float cur2 = cur1 / current_box.SurfaceArea();
			 
			 cost[i] = .125f + cur2;
				 
				 
				 
				 
				 
		 }

		 // Find bucket to split at that minimizes SAH metric
		 float minCost = cost[0];
		 int minCostSplit = 0;
		 for (int i = 1; i < nBuckets-1; ++i) 
		 {
			if (cost[i] < minCost) 
			{
				minCost = cost[i];
				minCostSplit = i;
			}
		 }

		 Previews *pmid = std::partition(&previews[PreviewsArray_start],
							&previews[PreviewsArray_end-1]+1,
							CompareToBucket(minCostSplit, nBuckets, axis, centroidBounds));
						mid = pmid - &previews[0];

						if(mid==140) 
							int lkjasdf=0;
		// Here, I have tried to change the mid just to the middle point, and it worked. So I think the only problem is the calculation of the mid
		// for the method of SAH.


		

		




		int start = PreviewsArray_start;
		
		int end = PreviewsArray_end;

		//mid = (start + end)/2;
		//mid = (start + end)/2;
        // Partition primitives into two sets and build children

		std::cout<<mid<<std::endl;

		node->InitInterior(axis,recursiveBuild(start, mid), recursiveBuild(mid, end));
		totalBVH_TreeNode++;
        return node;
	}
 }

      
 int countflat = 0;
       
 int flattenBVHTree(BVH_TreeNode *node, int *offset) 
 {
	
	 countflat++;
	 //std::cout<<countflat<<std::endl;
	// if(countflat ==133)
	//	 int kkkkk=3;

    LinearBVHNode *linearNode = &nodes[*offset];

    linearNode->bounds = node->bounds;
    int myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
       // Assert(!node->children[0] && !node->children[1]);
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    }
    else {
        // Creater interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset = flattenBVHTree(node->children[1],
                                                       offset);
    }
    return myOffset;
}










static void loadScene(char *name) {
	/* load the scene into the SceneIO data structure using given parsing code */
	scene = readScene(name);

	E[0] = scene->camera->position[0];
	E[1] = scene->camera->position[1];
	E[2] = scene->camera->position[2];

	V[0] = scene->camera->viewDirection[0];
	V[1] = scene->camera->viewDirection[1];
	V[2] = scene->camera->viewDirection[2];

	U[0] = scene->camera->orthoUp[0];
	U[1] = scene->camera->orthoUp[1];
	U[2] = scene->camera->orthoUp[2];

	phi = scene->camera->verticalFOV;

	U.normalize();
	V.normalize();

	//Calculate the X-axis direction
	A = V.cross(U);
	B = A.cross(V);
	A.normalize();
	B.normalize();

	M = E + c * V;

	//The Vertical vector Y
	Y = c * tan(phi/2.f) * B;

	//The Horizonal Vector 
	X = c * tan(phi/2.f * (IMAGE_WIDTH / IMAGE_HEIGHT)) * A;

	/* hint: use the Visual Studio debugger ("watch" feature) to probe the
	   scene data structure and learn more about it for each of the given scenes */


	/* write any code to transfer from the scene data structure to your own here */
	/* */

	return;
}





/* just a place holder, feel free to edit */
//void render(void) {
//	int i, j, k;
//	u08 *image = (u08 *)malloc(sizeof(u08) * IMAGE_HEIGHT * IMAGE_WIDTH * 3);
//	u08 *ptr = image;
//
//	for (j = 0; j < IMAGE_HEIGHT; j++) {
//		for (i = 0; i < IMAGE_WIDTH; i++) {
//			for (k = 0; k < 3; k++) {
//				*(ptr++) = 0;
//			}
//		}
//	}
//
//	/* save out the image */
//	/* */
//
//	/* cleanup */
//	free(image);
//
//	return;
//}



TraceResult RayTrace(Vector3f RayDir, Vector3f RayOri)
{
	
	MaterialIO* theMaterial = new MaterialIO;
	Vector3f theNormal;
	Vector3f thePosition;
	float Min = INFINITE;
	TraceResult result;

	ObjIO * FirstObj = scene->objects;

	while(scene->objects != NULL)
	{
		if(scene->objects->type == SPHERE_OBJ) //Sphere
		{
			SphereIO* sphere = NULL;
			sphere = (SphereIO*) (scene->objects->data);

			Vector3f SphereCenter;
			float Radius;
			

			SphereCenter[0] = sphere->origin[0];
			SphereCenter[1] = sphere->origin[1];
			SphereCenter[2] = sphere->origin[2];
			Radius = sphere->radius;


			/*float speed = 10;
			float Move = RandomFloat() * SHOT_TIME * speed;

			SphereCenter[0] += Move;
			SphereCenter[1] += Move;
			SphereCenter[2] += Move;*/




			float d = RaySphereIntersection(SphereCenter,  Radius,  RayDir,  RayOri);
			if (d != 0)
			{
				result.IntersectOrNot = 1;
				
				if(d<Min && d>0)
				{
					Min = d;
					//?????????????????????????????? what should be the normal of the sphere????????????????????????????????????????????????
					theNormal = (RayOri + Min * RayDir - SphereCenter) / Radius;
		
					theMaterial->ambColor[0] = scene->objects->material->ambColor[0];
					theMaterial->ambColor[1] = scene->objects->material->ambColor[1];
					theMaterial->ambColor[2] = scene->objects->material->ambColor[2];

					theMaterial->diffColor[0] = scene->objects->material->diffColor[0];
					theMaterial->diffColor[1] = scene->objects->material->diffColor[1];
					theMaterial->diffColor[2] = scene->objects->material->diffColor[2];

					theMaterial->ktran = scene->objects->material->ktran;
					theMaterial->shininess = scene->objects->material->shininess;

					theMaterial->specColor[0] = scene->objects->material->specColor[0];
					theMaterial->specColor[1] = scene->objects->material->specColor[1];
					theMaterial->specColor[2] = scene->objects->material->specColor[2];

					theMaterial->emissColor[0] = scene->objects->material->emissColor[0];
					theMaterial->emissColor[1] = scene->objects->material->emissColor[1];
					theMaterial->emissColor[2] = scene->objects->material->emissColor[2];
				}
			}	
		}
		else        //PolySet
		{
			PolySetIO* polySet = NULL;
			polySet = (PolySetIO*) (scene->objects->data);

			for(int i = 0; i < polySet->numPolys; i++)
			{
				PolygonIO poly;
				poly = polySet->poly[i];  
				
				Vector3f V0;
				Vector3f V1;
				Vector3f V2;
				VertexIO vertex;

				Vector3f V0_Normal;
				Vector3f V1_Normal;
				Vector3f V2_Normal;


				vertex = poly.vert[0];
				V0[0] = vertex.pos[0];
				V0[1] = vertex.pos[1];
				V0[2] = vertex.pos[2];
				

				vertex = poly.vert[1];
				V1[0] = vertex.pos[0];
				V1[1] = vertex.pos[1];
				V1[2] = vertex.pos[2];
				

				vertex = poly.vert[2];
				V2[0] = vertex.pos[0];
				V2[1] = vertex.pos[1];
				V2[2] = vertex.pos[2];
			

				Vector3f IntersectionDefine = RayTriangleIntersection(V0,V1,V2,RayDir,RayOri);
				if (IntersectionDefine(0)>0 && IntersectionDefine(1)>=0 && IntersectionDefine(2)>=0 && (IntersectionDefine(1) + IntersectionDefine(2))<=1)
				{
					Vector3f d = RayTriangleIntersection(V0,V1,V2,RayDir,RayOri); 
					if(d(0)<Min && d(0)>0)
					{
						if (polySet->normType == PER_VERTEX_NORMAL)
						{
							//Here I should also test if the poly's material is Per_Vertex or Per_Object

							float v = IntersectionDefine[1];
							float u = IntersectionDefine[2];
							float w = 1 - u - v;

					

							V0_Normal[0] = poly.vert[0].norm[0];
							V0_Normal[1] = poly.vert[0].norm[1];
							V0_Normal[2] = poly.vert[0].norm[2];

							V1_Normal[0] = poly.vert[1].norm[0];
							V1_Normal[1] = poly.vert[1].norm[1];
							V1_Normal[2] = poly.vert[1].norm[2];

							V2_Normal[0] = poly.vert[2].norm[0];
							V2_Normal[1] = poly.vert[2].norm[1];
							V2_Normal[2] = poly.vert[2].norm[2];

							result.IntersectOrNot = 1;
							Min = d(0);
							//theNormal = (V1-V0).cross(V2-V0);
							theNormal = u * V2_Normal + v * V1_Normal + w * V0_Normal;
					
							MaterialIO Material_0 = scene->objects->material[poly.vert[0].materialIndex];
							MaterialIO Material_1 = scene->objects->material[poly.vert[1].materialIndex];
							MaterialIO Material_2 = scene->objects->material[poly.vert[2].materialIndex];
							theMaterial->ambColor[0] = Material_0.ambColor[0] * w + Material_1.ambColor[0] * v + Material_2.ambColor[0] * u;
							theMaterial->ambColor[1] = Material_0.ambColor[1] * w + Material_1.ambColor[1] * v + Material_2.ambColor[1] * u;
							theMaterial->ambColor[2] = Material_0.ambColor[2] * w + Material_1.ambColor[2] * v + Material_2.ambColor[2] * u;

							theMaterial->diffColor[0] = Material_0.diffColor[0] * w + Material_1.diffColor[0] * v + Material_2.diffColor[0] * u;
							theMaterial->diffColor[1] = Material_0.diffColor[1] * w + Material_1.diffColor[1] * v + Material_2.diffColor[1] * u;
							theMaterial->diffColor[2] = Material_0.diffColor[2] * w + Material_1.diffColor[2] * v + Material_2.diffColor[2] * u;

							theMaterial->specColor[0] = Material_0.specColor[0] * w + Material_1.specColor[0] * v + Material_2.specColor[0] * u;
							theMaterial->specColor[1] = Material_0.specColor[1] * w + Material_1.specColor[1] * v + Material_2.specColor[1] * u;
							theMaterial->specColor[2] = Material_0.specColor[2] * w + Material_1.specColor[2] * v + Material_2.specColor[2] * u;

							theMaterial->emissColor[0] = Material_0.emissColor[0] * w + Material_1.emissColor[0] * v + Material_2.emissColor[0] * u;
							theMaterial->emissColor[1] = Material_0.emissColor[1] * w + Material_1.emissColor[1] * v + Material_2.emissColor[1] * u;
							theMaterial->emissColor[2] = Material_0.emissColor[2] * w + Material_1.emissColor[2] * v + Material_2.emissColor[2] * u;

							theMaterial->ktran = Material_0.ktran * w + Material_1.ktran * v + Material_2.ktran * u;
							theMaterial->shininess = Material_0.shininess * w + Material_1.shininess * v + Material_2.shininess * u;
						}
						if(polySet->normType == PER_FACE_NORMAL)
						{
							result.IntersectOrNot = 1;
							Min = d(0);
							theNormal = (V1-V0).cross(V2-V0);

							if (polySet->materialBinding == PER_VERTEX_MATERIAL)
							{
								float v = IntersectionDefine[1];
								float u = IntersectionDefine[2];
								float w = 1 - u - v;
							
								MaterialIO Material_0 = scene->objects->material[poly.vert[0].materialIndex];
								MaterialIO Material_1 = scene->objects->material[poly.vert[1].materialIndex];
								MaterialIO Material_2 = scene->objects->material[poly.vert[2].materialIndex];
								theMaterial->ambColor[0] = Material_0.ambColor[0] * w + Material_1.ambColor[0] * v + Material_2.ambColor[0] * u;
								theMaterial->ambColor[1] = Material_0.ambColor[1] * w + Material_1.ambColor[1] * v + Material_2.ambColor[1] * u;
								theMaterial->ambColor[2] = Material_0.ambColor[2] * w + Material_1.ambColor[2] * v + Material_2.ambColor[2] * u;

								theMaterial->diffColor[0] = Material_0.diffColor[0] * w + Material_1.diffColor[0] * v + Material_2.diffColor[0] * u;
								theMaterial->diffColor[1] = Material_0.diffColor[1] * w + Material_1.diffColor[1] * v + Material_2.diffColor[1] * u;
								theMaterial->diffColor[2] = Material_0.diffColor[2] * w + Material_1.diffColor[2] * v + Material_2.diffColor[2] * u;

								theMaterial->specColor[0] = Material_0.specColor[0] * w + Material_1.specColor[0] * v + Material_2.specColor[0] * u;
								theMaterial->specColor[1] = Material_0.specColor[1] * w + Material_1.specColor[1] * v + Material_2.specColor[1] * u;
								theMaterial->specColor[2] = Material_0.specColor[2] * w + Material_1.specColor[2] * v + Material_2.specColor[2] * u;

								theMaterial->emissColor[0] = Material_0.emissColor[0] * w + Material_1.emissColor[0] * v + Material_2.emissColor[0] * u;
								theMaterial->emissColor[1] = Material_0.emissColor[1] * w + Material_1.emissColor[1] * v + Material_2.emissColor[1] * u;
								theMaterial->emissColor[2] = Material_0.emissColor[2] * w + Material_1.emissColor[2] * v + Material_2.emissColor[2] * u;

								theMaterial->ktran = Material_0.ktran * w + Material_1.ktran * v + Material_2.ktran * u;
								theMaterial->shininess = Material_0.shininess * w + Material_1.shininess * v + Material_2.shininess * u;
							}
							else 
							{
								theMaterial->ambColor[0] = scene->objects->material->ambColor[0];
								theMaterial->ambColor[1] = scene->objects->material->ambColor[1];
								theMaterial->ambColor[2] = scene->objects->material->ambColor[2];

								theMaterial->diffColor[0] = scene->objects->material->diffColor[0];
								theMaterial->diffColor[1] = scene->objects->material->diffColor[1];
								theMaterial->diffColor[2] = scene->objects->material->diffColor[2];

								theMaterial->ktran = scene->objects->material->ktran;
								theMaterial->shininess = scene->objects->material->shininess;

								theMaterial->specColor[0] = scene->objects->material->specColor[0];
								theMaterial->specColor[1] = scene->objects->material->specColor[1];
								theMaterial->specColor[2] = scene->objects->material->specColor[2];

								theMaterial->emissColor[0] = scene->objects->material->emissColor[0];
								theMaterial->emissColor[1] = scene->objects->material->emissColor[1];
								theMaterial->emissColor[2] = scene->objects->material->emissColor[2];
							}
						}
					}
				}
			}
		}
		scene->objects = scene->objects->next;
	}	
	scene->objects = FirstObj;  


	thePosition = Min * RayDir + RayOri;
	
	result.Material = theMaterial;

	if(result.IntersectOrNot)
	{
		thePosition = Min * RayDir + RayOri;
		result.Position = thePosition;
		theNormal.normalize();
		result.Normal = theNormal;
		result.Min = Min;
	}
	
	return result;
}



TraceResult RayTrace_BVH(Vector3f RayDir, Vector3f RayOri)
{
	
	MaterialIO* theMaterial = new MaterialIO;
	Vector3f theNormal;
	Vector3f thePosition;
	float Min = FLT_MAX;
	TraceResult result;




	Vector3f invDir(1.f / RayDir[0], 1.f / RayDir[1], 1.f / RayDir[2]);
    int dirIsNeg[3] = { invDir[0] < 0, invDir[1] < 0, invDir[2] < 0 };

	int todoOffset = 0, nodeNum = 0;
    int todo[64];




	while (true) 
	{
		const LinearBVHNode *node = &nodes[nodeNum];
		// Check ray against BVH node
		if (Ray_Box_Interset(node->bounds, RayDir, RayOri)) 
		{
            if (node->nPrimitives > 0) 
			{ // Here Interset with the leaf Node;
                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i)
                {
					// Here I do not use the bounding box anymore, there are 10 primes, So I interset with them one by one!!!!!!!!!!!!!!!!!!
					// And I consider it is Sphere or Poly


					//Here is the most question, and important part!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					Previews cur_preview = previews[node->primitivesOffset+i];   // Here is the most important part! Is this right? Choose from previes??????????
					
					if (cur_preview.type == Sphere)  // sphere
                    {
						Vector3f SphereCenter;
						float Radius;
			

						SphereCenter[0] = cur_preview.SphereOri[0];
						SphereCenter[1] = cur_preview.SphereOri[1];
						SphereCenter[2] = cur_preview.SphereOri[2];
						Radius = cur_preview.radius;

						float speed = 10;
						float Move = RandomFloat() * SHOT_TIME * speed;

						SphereCenter[0] = cur_preview.SphereOri[0] + Move;
						SphereCenter[1] = cur_preview.SphereOri[1] + Move;
						SphereCenter[2] = cur_preview.SphereOri[2] + Move;




						float d = RaySphereIntersection(SphereCenter,  Radius,  RayDir,  RayOri);
						if (d != 0)
						{
							result.IntersectOrNot = 1;
				
							if(d<Min && d>0)
							{
								Min = d;
								theNormal = (RayOri + Min * RayDir - SphereCenter) / Radius;
		
								theMaterial->ambColor[0] = cur_preview.material->ambColor[0];
								theMaterial->ambColor[1] = cur_preview.material->ambColor[1];
								theMaterial->ambColor[2] = cur_preview.material->ambColor[2];

								theMaterial->diffColor[0] = cur_preview.material->diffColor[0];
								theMaterial->diffColor[1] = cur_preview.material->diffColor[1];
								theMaterial->diffColor[2] = cur_preview.material->diffColor[2];

								theMaterial->ktran = cur_preview.material->ktran;
								theMaterial->shininess = cur_preview.material->shininess;

								theMaterial->specColor[0] = cur_preview.material->specColor[0];
								theMaterial->specColor[1] = cur_preview.material->specColor[1];
								theMaterial->specColor[2] = cur_preview.material->specColor[2];

								theMaterial->emissColor[0] = cur_preview.material->emissColor[0];
								theMaterial->emissColor[1] = cur_preview.material->emissColor[1];
								theMaterial->emissColor[2] = cur_preview.material->emissColor[2];

							}
						}	
                    }
                    else  // poly
					{
                        Vector3f V0;
						Vector3f V1;
						Vector3f V2;
						VertexIO vertex;

						Vector3f V0_Normal;
						Vector3f V1_Normal;
						Vector3f V2_Normal;


						vertex = cur_preview.vert[0];
						V0[0] = vertex.pos[0];
						V0[1] = vertex.pos[1];
						V0[2] = vertex.pos[2];
				

						vertex = cur_preview.vert[1];
						V1[0] = vertex.pos[0];
						V1[1] = vertex.pos[1];
						V1[2] = vertex.pos[2];
				

						vertex = cur_preview.vert[2];
						V2[0] = vertex.pos[0];
						V2[1] = vertex.pos[1];
						V2[2] = vertex.pos[2];
						Vector3f IntersectionDefine = RayTriangleIntersection(V0,V1,V2,RayDir,RayOri);

						if (IntersectionDefine(0)>0 && IntersectionDefine(1)>=0 && IntersectionDefine(2)>=0 && (IntersectionDefine(1) + IntersectionDefine(2))<=1)
						{
							Vector3f d = RayTriangleIntersection(V0,V1,V2,RayDir,RayOri); 
							if(d(0)<Min && d(0)>0)
							{
								if (cur_preview.normType == PER_VERTEX_NORMAL)
								{
									//Here I should also test if the poly's material is Per_Vertex or Per_Object

									float v = IntersectionDefine[1];
									float u = IntersectionDefine[2];
									float w = 1 - u - v;

					

									V0_Normal[0] = cur_preview.vert[0].norm[0];
									V0_Normal[1] = cur_preview.vert[0].norm[1];
									V0_Normal[2] = cur_preview.vert[0].norm[2];

									V1_Normal[0] = cur_preview.vert[1].norm[0];
									V1_Normal[1] = cur_preview.vert[1].norm[1];
									V1_Normal[2] = cur_preview.vert[1].norm[2];

									V2_Normal[0] = cur_preview.vert[2].norm[0];
									V2_Normal[1] = cur_preview.vert[2].norm[1];
									V2_Normal[2] = cur_preview.vert[2].norm[2];

									result.IntersectOrNot = 1;
									Min = d(0);
									//theNormal = (V1-V0).cross(V2-V0);
									theNormal = u * V2_Normal + v * V1_Normal + w * V0_Normal;
									if (cur_preview.materialBinding == PER_VERTEX_MATERIAL)
									{
										MaterialIO Material_0 = cur_preview.material[cur_preview.vert[0].materialIndex];
										MaterialIO Material_1 = cur_preview.material[cur_preview.vert[1].materialIndex];
										MaterialIO Material_2 = cur_preview.material[cur_preview.vert[2].materialIndex];
										theMaterial->ambColor[0] = Material_0.ambColor[0] * w + Material_1.ambColor[0] * v + Material_2.ambColor[0] * u;
										theMaterial->ambColor[1] = Material_0.ambColor[1] * w + Material_1.ambColor[1] * v + Material_2.ambColor[1] * u;
										theMaterial->ambColor[2] = Material_0.ambColor[2] * w + Material_1.ambColor[2] * v + Material_2.ambColor[2] * u;

										theMaterial->diffColor[0] = Material_0.diffColor[0] * w + Material_1.diffColor[0] * v + Material_2.diffColor[0] * u;
										theMaterial->diffColor[1] = Material_0.diffColor[1] * w + Material_1.diffColor[1] * v + Material_2.diffColor[1] * u;
										theMaterial->diffColor[2] = Material_0.diffColor[2] * w + Material_1.diffColor[2] * v + Material_2.diffColor[2] * u;

										theMaterial->specColor[0] = Material_0.specColor[0] * w + Material_1.specColor[0] * v + Material_2.specColor[0] * u;
										theMaterial->specColor[1] = Material_0.specColor[1] * w + Material_1.specColor[1] * v + Material_2.specColor[1] * u;
										theMaterial->specColor[2] = Material_0.specColor[2] * w + Material_1.specColor[2] * v + Material_2.specColor[2] * u;

										theMaterial->emissColor[0] = Material_0.emissColor[0] * w + Material_1.emissColor[0] * v + Material_2.emissColor[0] * u;
										theMaterial->emissColor[1] = Material_0.emissColor[1] * w + Material_1.emissColor[1] * v + Material_2.emissColor[1] * u;
										theMaterial->emissColor[2] = Material_0.emissColor[2] * w + Material_1.emissColor[2] * v + Material_2.emissColor[2] * u;

										theMaterial->ktran = Material_0.ktran * w + Material_1.ktran * v + Material_2.ktran * u;
										theMaterial->shininess = Material_0.shininess * w + Material_1.shininess * v + Material_2.shininess * u;
									}
									else 
									{
										theMaterial =  cur_preview.material;
									}
								}
								if(cur_preview.normType == PER_FACE_NORMAL)
								{
									result.IntersectOrNot = 1;
									Min = d(0);
									theNormal = (V1-V0).cross(V2-V0);

									if (cur_preview.materialBinding == PER_VERTEX_MATERIAL)
									{
										float v = IntersectionDefine[1];
										float u = IntersectionDefine[2];
										float w = 1 - u - v;
							
										MaterialIO Material_0 = cur_preview.material[cur_preview.vert[0].materialIndex];
										MaterialIO Material_1 = cur_preview.material[cur_preview.vert[1].materialIndex];
										MaterialIO Material_2 = cur_preview.material[cur_preview.vert[2].materialIndex];
										theMaterial->ambColor[0] = Material_0.ambColor[0] * w + Material_1.ambColor[0] * v + Material_2.ambColor[0] * u;
										theMaterial->ambColor[1] = Material_0.ambColor[1] * w + Material_1.ambColor[1] * v + Material_2.ambColor[1] * u;
										theMaterial->ambColor[2] = Material_0.ambColor[2] * w + Material_1.ambColor[2] * v + Material_2.ambColor[2] * u;

										theMaterial->diffColor[0] = Material_0.diffColor[0] * w + Material_1.diffColor[0] * v + Material_2.diffColor[0] * u;
										theMaterial->diffColor[1] = Material_0.diffColor[1] * w + Material_1.diffColor[1] * v + Material_2.diffColor[1] * u;
										theMaterial->diffColor[2] = Material_0.diffColor[2] * w + Material_1.diffColor[2] * v + Material_2.diffColor[2] * u;

										theMaterial->specColor[0] = Material_0.specColor[0] * w + Material_1.specColor[0] * v + Material_2.specColor[0] * u;
										theMaterial->specColor[1] = Material_0.specColor[1] * w + Material_1.specColor[1] * v + Material_2.specColor[1] * u;
										theMaterial->specColor[2] = Material_0.specColor[2] * w + Material_1.specColor[2] * v + Material_2.specColor[2] * u;

										theMaterial->emissColor[0] = Material_0.emissColor[0] * w + Material_1.emissColor[0] * v + Material_2.emissColor[0] * u;
										theMaterial->emissColor[1] = Material_0.emissColor[1] * w + Material_1.emissColor[1] * v + Material_2.emissColor[1] * u;
										theMaterial->emissColor[2] = Material_0.emissColor[2] * w + Material_1.emissColor[2] * v + Material_2.emissColor[2] * u;

										theMaterial->ktran = Material_0.ktran * w + Material_1.ktran * v + Material_2.ktran * u;
										theMaterial->shininess = Material_0.shininess * w + Material_1.shininess * v + Material_2.shininess * u;
									}
									else 
									{
										theMaterial->ambColor[0] = cur_preview.material->ambColor[0];
										theMaterial->ambColor[1] = cur_preview.material->ambColor[1];
										theMaterial->ambColor[2] = cur_preview.material->ambColor[2];

										theMaterial->diffColor[0] = cur_preview.material->diffColor[0];
										theMaterial->diffColor[1] = cur_preview.material->diffColor[1];
										theMaterial->diffColor[2] = cur_preview.material->diffColor[2];

										theMaterial->ktran = cur_preview.material->ktran;
										theMaterial->shininess = cur_preview.material->shininess;

										theMaterial->specColor[0] = cur_preview.material->specColor[0];
										theMaterial->specColor[1] = cur_preview.material->specColor[1];
										theMaterial->specColor[2] = cur_preview.material->specColor[2];

										theMaterial->emissColor[0] = cur_preview.material->emissColor[0];
										theMaterial->emissColor[1] = cur_preview.material->emissColor[1];
										theMaterial->emissColor[2] = cur_preview.material->emissColor[2];
									}
								}
							}
						}
                    }
              
				
				}
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
            else 
			{
                // Put far BVH node on _todo_ stack, advance to near node
                if (dirIsNeg[node->axis]) 
				{
                   todo[todoOffset++] = nodeNum + 1;
                   nodeNum = node->secondChildOffset;
                }
                else 
				{
                   todo[todoOffset++] = node->secondChildOffset;
                   nodeNum = nodeNum + 1;
                }
            }
        }
        else {
            if (todoOffset == 0) break;
            nodeNum = todo[--todoOffset];
        }
    }


	result.Material = theMaterial;

	if(result.IntersectOrNot)
	{
		thePosition = Min * RayDir + RayOri;
		result.Position = thePosition;
		theNormal.normalize();
		result.Normal = theNormal;
		result.Min = Min;
	}
	
	return result;
}




Vector3f Reflection (float Sr, float Sg, float Sb, Vector3f ReflectionDir, Vector3f Position)
{
	Vector3f Refresult;
	traceTime++;
	if(traceTime >= TrT)
	{
		Refresult[0] = 0;
		Refresult[1] = 0;
		Refresult[2] = 0;
		return Refresult;
	
	}


	if(Sr == 0 && Sg == 0 && Sb == 0)
		{
		Refresult[0] = 0;
		Refresult[1] = 0;
		Refresult[2] = 0;
		}
	else
			{
			TraceResult result;
			result = RayTrace_BVH(ReflectionDir,Position);

			if (result.IntersectOrNot)
			{
				//Shadding
				Vector3f Normal = result.Normal;
				float Ka,Kd,Ks,Ar,Ag,Ab,Dr,Dg,Db,Sr,Sg,Sb,kt;
				// Self Definition
				Ka = 1;
				Ks = 1;
				Kd = 1;


				
				// Load the Material Data
				float q = result.Material->shininess*128;

				kt = result.Material->ktran; //Transparancy

				Ar = result.Material->ambColor[0];
				Ag = result.Material->ambColor[1];
				Ab = result.Material->ambColor[2];

				Dr = result.Material->diffColor[0];
				Dg = result.Material->diffColor[1];
				Db = result.Material->diffColor[2];

				Sr = result.Material->specColor[0];
				Sg = result.Material->specColor[1];
				Sb = result.Material->specColor[2];


				//Loop the light
				float DirectIllr,DirectIllg,DirectIllb;
				DirectIllr = DirectIllg = DirectIllb = 0;

				LightIO * FirstLight = scene->lights;
				Vector3f R;

				Vector3f view = -ReflectionDir;
				view.normalize();

				while ( scene->lights != NULL)
				{
					Vector3f LightPosition;
					Vector3f LightDir;
					float t;
					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						LightDir[0] = -scene->lights->direction[0];
						LightDir[1] = -scene->lights->direction[1];
						LightDir[2] = -scene->lights->direction[2];
					}
					
					else if( scene->lights->type == POINT_LIGHT )
					{
						LightPosition[0] = scene->lights->position[0];
						LightPosition[1] = scene->lights->position[1];
						LightPosition[2] = scene->lights->position[2];

						LightDir = LightPosition-result.Position;
					}

					LightDir.normalize();


					float Lr = scene->lights->color[0];
					float Lg = scene->lights->color[1];
					float Lb = scene->lights->color[2];


					R = 2 * Normal.dot(LightDir) * Normal - LightDir;
					R.normalize();
					Vector3f shadDir = LightDir;

					//Why it is minus here????????????????????????????????????????????????????????????????????????????????????????????????
					TraceResult LightResult = RayTrace_BVH(shadDir,result.Position + MOVE_FACTOR * shadDir);
					float fattj;
					float S=1;
					if (scene->lights->type == POINT_LIGHT)
					{
						float x1,x2,x3;
						x1 = LightPosition[0] - result.Position[0];
						x2 = LightPosition[1] - result.Position[1];
						x3 = LightPosition[2] - result.Position[2];
						t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

						if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)
						{
							S = 0;
						}
						else
						{
							S = 1;
						}
						fattj = min(1, 1/(0.25 + 0.1 * t + 0.01 * pow(t,2)));
					}

					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						if(LightResult.IntersectOrNot && LightResult.Min >= 0)
						{
							S = 0;
						}
						else
						{
							S = 1;
						}
						fattj = 1;
					}
					
					float kkk = (Normal.dot(LightDir));
					kkk = (kkk<0) ? 0:kkk;

					// Why I set the q to be 0.2 will cause bugs? The S is 0, why can not the plus result be 0?????????????????????????????????????????
					DirectIllr =(S * Lr* fattj * ((1-kt) * kkk * Dr + Sr*pow(clamp(R.dot(view)),q))) + DirectIllr;
					DirectIllg =(S * Lg* fattj * ((1-kt) * kkk * Dg + Sg*pow(clamp(R.dot(view)),q))) + DirectIllg;
					DirectIllb =(S * Lb* fattj * ((1-kt) * kkk * Db + Sb*pow(clamp(R.dot(view)),q))) + DirectIllb;

					scene->lights = scene->lights->next;
				}

				scene->lights = FirstLight;

				Vector3f RView = 2 * Normal.dot(view) * Normal - view;
				RView.normalize();

				Vector3f RefractionDir_next = getRefractionDir(result.Normal,ReflectionDir,1,1.5);
				Vector3f ReflectionResult_next = Reflection(Sr,Sg,Sb, RView, result.Position + MOVE_FACTOR*RView);
				Vector3f RefractionRes = Refraction(kt, RefractionDir_next, result.Position + MOVE_FACTOR * RefractionDir_next);
				// Why negative??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
				Refresult[0] = (((1-kt) * Ar * Dr + DirectIllr) + Sr * ReflectionResult_next[0] + kt * RefractionRes[0]);
				Refresult[1] = (((1-kt) * Ag * Dg + DirectIllg) + Sg * ReflectionResult_next[1] + kt * RefractionRes[1]);
				Refresult[2] = (((1-kt) * Ab * Db + DirectIllb) + Sb * ReflectionResult_next[2] + kt * RefractionRes[2]);

				
			}
		else
			{
			Refresult[0] = 0;
			Refresult[1] = 0;
			Refresult[2] = 0;
		
			}


		}
		
		return Refresult;
}

Vector3f Refraction(float ktran, Vector3f RefractionDir, Vector3f Position)
{
	Vector3f Refresult;
	traceTime++;
	if(traceTime >= TrT)
	{
		Refresult[0] = 0;
		Refresult[1] = 0;
		Refresult[2] = 0;
		return Refresult;
	}


	if(ktran == 0)
		{
		Refresult[0] = 0;
		Refresult[1] = 0;
		Refresult[2] = 0;
		}
	else
		{
			TraceResult result;
			result = RayTrace_BVH(RefractionDir,Position);

			if (result.IntersectOrNot)
			{
				//Shadding
				Vector3f Normal = result.Normal;
				float Ka,Kd,Ks,Ar,Ag,Ab,Dr,Dg,Db,Sr,Sg,Sb,kt;
				// Self Definition
				Ka = 1;
				Ks = 1;
				Kd = 1;


				
				// Load the Material Data
				float q = result.Material->shininess * 128;

				kt = result.Material->ktran; //Transparancy

				Ar = result.Material->ambColor[0];
				Ag = result.Material->ambColor[1];
				Ab = result.Material->ambColor[2];

				Dr = result.Material->diffColor[0];
				Dg = result.Material->diffColor[1];
				Db = result.Material->diffColor[2];

				Sr = result.Material->specColor[0];
				Sg = result.Material->specColor[1];
				Sb = result.Material->specColor[2];


				//Loop the light
				float DirectIllr,DirectIllg,DirectIllb;
				DirectIllr = DirectIllg = DirectIllb = 0;

				LightIO * FirstLight = scene->lights;
				Vector3f R;
				//????????????????????????????????????????????????????????????????????????????????????????????
				Vector3f view = -RefractionDir;//-----------------------------
				view.normalize();

				while ( scene->lights != NULL)
				{
					Vector3f LightPosition;
					Vector3f LightDir;
					float t;
					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						LightDir[0] = -scene->lights->direction[0];
						LightDir[1] = -scene->lights->direction[1];
						LightDir[2] = -scene->lights->direction[2];
					}
					
					else if( scene->lights->type == POINT_LIGHT )
					{
						LightPosition[0] = scene->lights->position[0];
						LightPosition[1] = scene->lights->position[1];
						LightPosition[2] = scene->lights->position[2];

						LightDir = LightPosition - result.Position;
					}

					LightDir.normalize();


					float Lr = scene->lights->color[0];
					float Lg = scene->lights->color[1];
					float Lb = scene->lights->color[2];


					R = 2 * Normal.dot(LightDir) * Normal - LightDir;
					R.normalize();

					Vector3f shadDir = LightDir;

					//Why it is minus here????????????????????????????????????????????????????????????????????????????????????????????????
					TraceResult LightResult = RayTrace_BVH(shadDir,result.Position + MOVE_FACTOR * shadDir);
					float fattj;
					float ssr,ssg,ssb,S;
					ssr = ssg = ssb = S = 1;

					//Vector3f color_rgb  = Normalize_color(LightResult.Material->diffColor[0],LightResult.Material->diffColor[1],LightResult.Material->diffColor[2]);

					if (scene->lights->type == POINT_LIGHT)
					{
						float x1,x2,x3;
						x1 = LightPosition[0] - result.Position[0];
						x2 = LightPosition[1] - result.Position[1];
						x3 = LightPosition[2] - result.Position[2];
						t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

						if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)
						{
							Vector3f color_rgb  = Normalize_color(LightResult.Material->diffColor[0],LightResult.Material->diffColor[1],LightResult.Material->diffColor[2]);

							S = 0;
							ssr = (LightResult.Material->ktran) * ssr * color_rgb[0];
							ssg = (LightResult.Material->ktran) * ssg * color_rgb[1];
							ssb = (LightResult.Material->ktran) * ssb * color_rgb[2];
						}
						else
						{
							S = 1;
						}
						fattj = min(1, 1/(0.25 + 0.1 * t + 0.01 * pow(t,2)));
					}

					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						if(LightResult.IntersectOrNot && LightResult.Min >= 0)
						{
							Vector3f color_rgb  = Normalize_color(LightResult.Material->diffColor[0],LightResult.Material->diffColor[1],LightResult.Material->diffColor[2]);

							S = 0;

							ssr = (LightResult.Material->ktran) * ssr * color_rgb[0];
							ssg = (LightResult.Material->ktran) * ssg * color_rgb[1];
							ssb = (LightResult.Material->ktran) * ssb * color_rgb[2];

						}
						else
						{
							S = 1;
						}
						fattj = 1;
					}
					
					float kkk = (Normal.dot(LightDir));
					kkk = (kkk<0) ? 0:kkk;
					
					
					DirectIllr =(ssr * Lr* fattj * ((1-kt) * kkk * Dr + Ks*Sr*pow(clamp(R.dot(view)),q))) + DirectIllr;
					DirectIllg =(ssg * Lg* fattj * ((1-kt) * kkk * Dg + Ks*Sg*pow(clamp(R.dot(view)),q))) + DirectIllg;
					DirectIllb =(ssb * Lb* fattj * ((1-kt) * kkk * Db + Ks*Sb*pow(clamp(R.dot(view)),q))) + DirectIllb;

					scene->lights = scene->lights->next;
				}

				scene->lights = FirstLight;

				//Calculate the View direction of the reflection
				

				Vector3f RefractionRes;
				float define = result.Normal.dot(-RefractionDir);
				float theta1 = std::acos(std::abs(result.Normal.dot(-RefractionDir)));

				if (define < 0)
					Normal = -Normal;


				if (define < 0 && (sin(theta1))*(1.5) >= 1) 
				{
					//Normal = -Normal;
					Vector3f newView = 2 * (Normal).dot(view) * (Normal) - view;
					RefractionRes=Reflection (Sr,Sg,Sb, newView, result.Position + 0.001 * newView); 
					RefractionRes.Zero();
				}
				else
				{
					Vector3f RefractionDir_next = getRefractionDir(result.Normal,RefractionDir,1,1.5);
					RefractionRes = Refraction(kt, RefractionDir_next, result.Position + MOVE_FACTOR * RefractionDir_next);
				}

				Vector3f RView = 2 * Normal.dot(view) * Normal - view;
				RView.normalize();
				Vector3f ReflectionRes = Reflection(Sr,Sg,Sb, RView, result.Position + MOVE_FACTOR * RView);
				
				Refresult[0] = (((1-kt) * Ar * Dr + DirectIllr) + Ks * ReflectionRes[0] + kt * RefractionRes[0]);
				Refresult[1] = (((1-kt) * Ag * Dg + DirectIllg) + Ks * ReflectionRes[1] + kt * RefractionRes[1]);
				Refresult[2] = (((1-kt) * Ab * Db + DirectIllb) + Ks * ReflectionRes[2] + kt * RefractionRes[2]);

				
			}
		else
			{
			Refresult[0] = 0;
			Refresult[1] = 0;
			Refresult[2] = 0;
		
			}


		}

		return Refresult;
}


Vector3f isecpt(TraceResult result, Vector3f refractionDir)
{

	Vector3f rgb_result;

	if (result.IntersectOrNot)
	{
		//Shadding
		Vector3f Normal = result.Normal;
		float Ar,Ag,Ab,Dr,Dg,Db,Sr,Sg,Sb,kt,Er;
		// Load the Material Data
		float q = result.Material->shininess*128;
		kt = result.Material->ktran; //Transparancy
		Ar = result.Material->ambColor[0];
		Ag = result.Material->ambColor[1];
		Ab = result.Material->ambColor[2];
		Dr = result.Material->diffColor[0];
		Dg = result.Material->diffColor[1];
		Db = result.Material->diffColor[2];
		Sr = result.Material->specColor[0];
		Sg = result.Material->specColor[1];
		Sb = result.Material->specColor[2];

		Er = result.Material->emissColor[2];

		//Loop the light
		float DirectIllr,DirectIllg,DirectIllb;
		DirectIllr = DirectIllg = DirectIllb = 0;

		LightIO * FirstLight = scene->lights;
		Vector3f R;

		Vector3f view;
		if(kt == 0)
		{
			view = E-result.Position;
			view.normalize();
		}
		else
		{
			view = -refractionDir;
			view.normalize();
		}

		while ( scene->lights != NULL)
		{
			Vector3f LightPosition;
			Vector3f LightDir;

			float t;
			if (scene->lights->type == DIRECTIONAL_LIGHT)
			{
				LightDir[0] = -scene->lights->direction[0];
				LightDir[1] = -scene->lights->direction[1];
				LightDir[2] = -scene->lights->direction[2];
			}
					
			else if( scene->lights->type == POINT_LIGHT )
			{
				LightPosition[0] = scene->lights->position[0];
				LightPosition[1] = scene->lights->position[1];
				LightPosition[2] = scene->lights->position[2];


				// Change Point Light to Square Light

				LightPosition[0] += RandomFloat() * 1 -0.5;
				LightPosition[2] += RandomFloat() * 1 -0.5;


				LightDir = LightPosition - result.Position;
			}
			LightDir.normalize();

			float Lr = scene->lights->color[0];
			float Lg = scene->lights->color[1];
			float Lb = scene->lights->color[2];

			R = 2 * Normal.dot(LightDir) * Normal - LightDir;
			R.normalize();
			Vector3f shadDir = LightDir;
		
			TraceResult LightResult = RayTrace(shadDir,result.Position + 0.001 * shadDir);
			float fattj;
			float S=1;
			Vector3f lightDirection_(0,1,0);
			lightDirection_.normalize();


			if (scene->lights->type == POINT_LIGHT)
			{
				float x1,x2,x3;
				x1 = LightPosition[0] - result.Position[0];
				x2 = LightPosition[1] - result.Position[1];
				x3 = LightPosition[2] - result.Position[2];
				t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

				if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)	S = 0;
				else 
				{
					S = 1;

					Vector3f ori = result.Position;
					Vector3f des = LightPosition;

					float dist_sq = (ori[0] - des[0]) * (ori[0] - des[0]) + (ori[1] - des[1]) * (ori[1] - des[1]) + (ori[2] - des[2]) * (ori[2] - des[2]);
					float dist = pow(dist_sq, 0.5);
					float absorption=exp(-sigma_t * 5 * dist);
					
					//new//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					float temporary = shadDir.dot(lightDirection_);
					if((temporary>=0.08) || temporary<1.1)
					S = S + 1;
					//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
					S = S * absorption;

					


					
				}
				fattj = min(1, 1/(0.25 + 0.1 * t + 0.01 * pow(t,2)));
			}







			float tempcos = shadDir.dot(lightDirection_);
			if((tempcos<=0.01) || tempcos>1.1)
				S=0;







			if (scene->lights->type == DIRECTIONAL_LIGHT)
			{
				if(LightResult.IntersectOrNot && LightResult.Min >= 0) S = 0;
				else S = 1;
				fattj = 1;
			}

			delete LightResult.Material;
			//LightResult.~TraceResult();
			//
			
			float kkk = (Normal.dot(LightDir));
			kkk = (kkk<0) ? 0:kkk;

			DirectIllr = (S * Lr* fattj * ((1-kt) * kkk * Dr + Sr*pow(clamp(R.dot(view)),q))) + DirectIllr;
			DirectIllg = (S * Lg* fattj * ((1-kt) * kkk * Dg + Sg*pow(clamp(R.dot(view)),q))) + DirectIllg;
			DirectIllb = (S * Lb* fattj * ((1-kt) * kkk * Db + Sb*pow(clamp(R.dot(view)),q))) + DirectIllb;

			scene->lights = scene->lights->next;
		}

		scene->lights = FirstLight;
		//traceTime = 0;

		rgb_result[0] = (((1-kt) * Ar * Dr + DirectIllr));
		rgb_result[1] = (((1-kt) * Ag * Dg + DirectIllg));
		rgb_result[2] = (((1-kt) * Ab * Db + DirectIllb));
	}

	else   // Here, if not intersect with the objects, then we need to add the environment map color!
	{
		// Direction --> Angular map coordinate conversion
		// Courtesy of Paul Debevec and Dan Lemmon's
		// SIGGRAPH 2001 Image-Based Lighting course notes

		Vector3f dir = refractionDir;

		double r = 0.159154943*acos(dir[2])/sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
		double u = 0.5 + dir[0] * r;
		double v = 0.5 + dir[1] * r;

		// Now get the radiance out of the HDR map
		int width = hdrImage->mSizeX;
		int height = hdrImage->mSizeY;


		int col = (int)(u*width);
		int row = (int)((1-v)*height);

		//rgb_result[0] = hdrImage->data[row*height + col].r;
		//rgb_result[1] = hdrImage->data[row*height + col].g;
		//rgb_result[2] = hdrImage->data[row*height + col].b;

		rgb_result[0] = 0;
		rgb_result[1] = 0;
		rgb_result[2] = 0;
	}

	return rgb_result;
}

class Virtual_Light
{
public:
	Vector3f position;
	Vector3f color;
	Vector3f Dir;
};

Vector3f isecpt_Bir(TraceResult result, Vector3f refractionDir)
{

	Vector3f rgb_result;

	if (result.IntersectOrNot)
	{
		//Shadding
		Vector3f Normal = result.Normal;
		float Ar,Ag,Ab,Dr,Dg,Db,Sr,Sg,Sb,kt;
		// Load the Material Data
		float q = result.Material->shininess*128;
		kt = result.Material->ktran; //Transparancy
		Ar = result.Material->ambColor[0];
		Ag = result.Material->ambColor[1];
		Ab = result.Material->ambColor[2];
		Dr = result.Material->diffColor[0];
		Dg = result.Material->diffColor[1];
		Db = result.Material->diffColor[2];
		Sr = result.Material->specColor[0];
		Sg = result.Material->specColor[1];
		Sb = result.Material->specColor[2];

		//Loop the light
		float DirectIllr,DirectIllg,DirectIllb;
		DirectIllr = DirectIllg = DirectIllb = 0;

		LightIO * FirstLight = scene->lights;
		Vector3f R;

		Vector3f view;
		if(kt == 0)
		{
			view = E-result.Position;
			view.normalize();
		}
		else
		{
			view = -refractionDir;
			view.normalize();
		}

		while ( scene->lights != NULL)
		{
			Vector3f LightPosition;
			Vector3f LightDir;

			Virtual_Light vlight_1, vlight_2, vlight_3, vlight_4;

			

			


			float t, Lr, Lg, Lb;
			if (scene->lights->type == DIRECTIONAL_LIGHT)
			{
				LightDir[0] = -scene->lights->direction[0];
				LightDir[1] = -scene->lights->direction[1];
				LightDir[2] = -scene->lights->direction[2];

				Lr = scene->lights->color[0];
				Lg = scene->lights->color[1];
				Lb = scene->lights->color[2];
				LightDir.normalize();
			}
					
			else if( scene->lights->type == POINT_LIGHT )
			{
				LightPosition[0] = scene->lights->position[0];
				LightPosition[1] = scene->lights->position[1];
				LightPosition[2] = scene->lights->position[2];


				// Change Point Light to Square Light

				LightPosition[0] += RandomFloat() * 1 -0.5;
				LightPosition[2] += RandomFloat() * 1 -0.5;


				vlight_1.Dir = randomAllDir();

				LightDir = LightPosition - result.Position;

				Lr = scene->lights->color[0];
				Lg = scene->lights->color[1];
				Lb = scene->lights->color[2];
				LightDir.normalize();


				TraceResult Vir_LightResult_1 = RayTrace(vlight_1.Dir,LightPosition);
				vlight_1.position = Vir_LightResult_1.Position + 0.1 * Vir_LightResult_1.Normal;
				


				Vector3f Dir_2;
				Dir_2 = 2 * Vir_LightResult_1.Normal.dot(-vlight_1.Dir) * Vir_LightResult_1.Normal + vlight_1.Dir;
				Dir_2.normalize();
				

				TraceResult Vir_LightResult_2 = RayTrace(Dir_2,vlight_1.position + 0.001 * Dir_2);
				vlight_2.position = Vir_LightResult_2.Position + 0.1 * Vir_LightResult_2.Normal;
				
				
				

				Vector3f Dir_3 = 2 * Vir_LightResult_2.Normal.dot(-Dir_2) * Vir_LightResult_2.Normal + Dir_2;
				Dir_3.normalize();
				TraceResult Vir_LightResult_3 = RayTrace(Dir_3,vlight_2.position + 0.001 * Dir_3);
				vlight_3.position = Vir_LightResult_3.Position + 0.1 * Vir_LightResult_3.Normal;
				

				Vector3f Dir_4 = 2 * Vir_LightResult_3.Normal.dot(-Dir_3) * Vir_LightResult_3.Normal + Dir_3;
				Dir_4.normalize();
				TraceResult Vir_LightResult_4 = RayTrace(Dir_4,vlight_3.position + 0.001 * Dir_4);
				vlight_4.position = Vir_LightResult_4.Position + 0.1 * Vir_LightResult_4.Normal;
				
				delete Vir_LightResult_1.Material;
				delete Vir_LightResult_2.Material;
				delete Vir_LightResult_3.Material;
				delete Vir_LightResult_4.Material;
			}
			

			R = 2 * Normal.dot(LightDir) * Normal - LightDir;
			R.normalize();
			Vector3f shadDir = LightDir;
		




			TraceResult LightResult = RayTrace(shadDir,result.Position + 0.001 * shadDir);
			float fattj;
			float S=1;
			if (scene->lights->type == POINT_LIGHT)
			{
				float x1,x2,x3;
				x1 = LightPosition[0] - result.Position[0];
				x2 = LightPosition[1] - result.Position[1];
				x3 = LightPosition[2] - result.Position[2];
				t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

				if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)
				{
					S = 0;
					Vector3f dir = vlight_1.position - result.Position;
					dir.normalize();

					TraceResult LightResult_vir = RayTrace(dir,result.Position + 0.001 * dir);

					x1 = vlight_1.position[0] - result.Position[0];
					x2 = vlight_1.position[1] - result.Position[1];
					x3 = vlight_1.position[2] - result.Position[2];
					t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

					if(LightResult_vir.IntersectOrNot == 1 && LightResult_vir.Min>=0 && LightResult_vir.Min <= t)
						S = S;
					else S += 1/(4*PI); 

					dir = vlight_2.position - result.Position;
					dir.normalize();


					x1 = vlight_2.position[0] - result.Position[0];
					x2 = vlight_2.position[1] - result.Position[1];
					x3 = vlight_2.position[2] - result.Position[2];
					t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

					delete LightResult_vir.Material;


					LightResult_vir = RayTrace(dir,result.Position + 0.001 * dir);
					if(LightResult_vir.IntersectOrNot == 1 && LightResult_vir.Min>=0 && LightResult_vir.Min <= t)
						S = S;
					else S += 1/(4*PI); 

					dir = vlight_3.position - result.Position;
					dir.normalize();

					x1 = vlight_3.position[0] - result.Position[0];
					x2 = vlight_3.position[1] - result.Position[1];
					x3 = vlight_3.position[2] - result.Position[2];
					t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

					delete LightResult_vir.Material;

					LightResult_vir = RayTrace(dir,result.Position + 0.001 * dir);
					if(LightResult_vir.IntersectOrNot == 1 && LightResult_vir.Min>=0 && LightResult_vir.Min <= t)
						S = S;
					else S += 1/(4*PI); 

					dir = vlight_4.position - result.Position;
					dir.normalize();

					x1 = vlight_4.position[0] - result.Position[0];
					x2 = vlight_4.position[1] - result.Position[1];
					x3 = vlight_4.position[2] - result.Position[2];
					t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

					delete LightResult_vir.Material;

					LightResult_vir = RayTrace(dir,result.Position + 0.001 * dir);
					if(LightResult_vir.IntersectOrNot == 1 && LightResult_vir.Min>=0 && LightResult_vir.Min <= t)
						S = S;
					else S += 1/(4*PI); 

					delete LightResult_vir.Material;
				}
				else S = 1;
				fattj = min(1, 1/(0.25 + 0.1 * t + 0.01 * pow(t,2)));
			}

			if (scene->lights->type == DIRECTIONAL_LIGHT)
			{
				if(LightResult.IntersectOrNot && LightResult.Min >= 0) S = 0;
				else S = 1;
				fattj = 1;
			}



			delete LightResult.Material;
			//LightResult.~TraceResult();
			//
			











			float kkk = (Normal.dot(LightDir));
			kkk = (kkk<0) ? 0:kkk;

			DirectIllr = (S * Lr* fattj * ((1-kt) * kkk * Dr + Sr*pow(clamp(R.dot(view)),q))) + DirectIllr;
			DirectIllg = (S * Lg* fattj * ((1-kt) * kkk * Dg + Sg*pow(clamp(R.dot(view)),q))) + DirectIllg;
			DirectIllb = (S * Lb* fattj * ((1-kt) * kkk * Db + Sb*pow(clamp(R.dot(view)),q))) + DirectIllb;

			scene->lights = scene->lights->next;
		}

		scene->lights = FirstLight;
		//traceTime = 0;

		rgb_result[0] = (((1-kt) * Ar * Dr + DirectIllr));
		rgb_result[1] = (((1-kt) * Ag * Dg + DirectIllg));
		rgb_result[2] = (((1-kt) * Ab * Db + DirectIllb));
	}

	else   // Here, if not intersect with the objects, then we need to add the environment map color!
	{
		// Direction --> Angular map coordinate conversion
		// Courtesy of Paul Debevec and Dan Lemmon's
		// SIGGRAPH 2001 Image-Based Lighting course notes

		Vector3f dir = refractionDir;

		double r = 0.159154943*acos(dir[2])/sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
		double u = 0.5 + dir[0] * r;
		double v = 0.5 + dir[1] * r;

		// Now get the radiance out of the HDR map
		int width = hdrImage->mSizeX;
		int height = hdrImage->mSizeY;


		int col = (int)(u*width);
		int row = (int)((1-v)*height);

		//rgb_result[0] = hdrImage->data[row*height + col].r;
		//rgb_result[1] = hdrImage->data[row*height + col].g;
		//rgb_result[2] = hdrImage->data[row*height + col].b;

		rgb_result[0] = 0;
		rgb_result[1] = 0;
		rgb_result[2] = 0;
	}

	return rgb_result;
}



Vector3f Vectormultipal(Vector3f a, Vector3f b)
{
	Vector3f _result;

	_result[0] = a[0] * b[0];
	_result[1] = a[1] * b[1];
	_result[2] = a[2] * b[2];

	return _result;
}



Vector3f PathTrace(Vector3f RView, Vector3f position) 
{
	
	Vector3f pathTrace_result(0,0,0);   
	Vector3f pathThroughput(1,1,1);
	TraceResult Traceresult;
	Vector3f outGoingRayDir = RView;
	Vector3f Position = position;

	for (int bounces = 0; bounces <= MAX_DEPTH; ++bounces)      // begin the loop
	{
		float Sr,Sg,Sb,kt,Er;
		Ray now;

		now.Ori = Position;

		Traceresult = RayTrace(outGoingRayDir,Position + MOVE_FACTOR * outGoingRayDir);
		

		kt = Traceresult.Material->ktran; 
		Sr = Traceresult.Material->specColor[0];
		Sg = Traceresult.Material->specColor[1];
		Sb = Traceresult.Material->specColor[2];
		Er = 0;
		Er = Traceresult.Material->emissColor[2];

		Position = Traceresult.Position;


		now.Destination = Position;


		Vector3f view = E-Traceresult.Position;
		view.normalize();


		Vector3f color = isecpt(Traceresult, outGoingRayDir);
		Vector3f one(1,1,1);
		now.Illumination = color;

		if(Er>0.5)
			now.Illumination = one;

		Er = 0;

		Vector3f _LightPosition;
		_LightPosition[0] = scene->lights->position[0];
		_LightPosition[1] = scene->lights->position[1];
		_LightPosition[2] = scene->lights->position[2];


		now.Dir = now.Destination - now.Ori;
		Vector3f _color;
		
			_color = ViewResult(now,_LightPosition,one);
		

		pathTrace_result = Vectormultipal(pathThroughput , _color) + pathTrace_result;  // Here, calcualte the rgb value;
		
		

		Vector3f Normal = Traceresult.Normal;

		delete Traceresult.Material;



		//Traceresult.~TraceResult();
		
		if(Sr!=0 || Sg!=0 || Sb!=0 || kt>0)   // First consider if it is Mirrow feflection
		{
			if(kt > 0)  // Test if it is transparant
			{

				pathThroughput[0] = kt;
				pathThroughput[1] = kt;
				pathThroughput[2] = kt;


				view = -outGoingRayDir;

				Vector3f RefractionDir_next = getRefractionDir(Normal,-view,1,1.5);
				outGoingRayDir = RefractionDir_next;
				outGoingRayDir.normalize();
			}

			if(kt ==0)
			{
				pathThroughput[0] = Sr;
				pathThroughput[1] = Sg;
				pathThroughput[2] = Sb;

				// Here, Consider the reflection Direction is Wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				// If there are more than one Specular object, there will be wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				outGoingRayDir = 2 * Normal.dot(view) * Normal - view;
				outGoingRayDir.normalize();

				// Here, the reflection effect should be consider.
				//pathTrace_result += Vectormultipal(pathThroughput, Reflection(Sr,Sg,Sb, outGoingRayDir, Traceresult.Position + MOVE_FACTOR * outGoingRayDir));
			}

		}
		else   // If not reflect, nor transparent, just do the Difuse Sample Path Trace
		{
			//这里我随即生成三个数字，这样我的取样，是在半球上平均分布吗？
			Vector3f DiffuseDir = randomDir();
			Vector3f vertical = getVertical(Normal);
			vertical.normalize();
			Vector3f lastOne = vertical.cross(Normal);
			lastOne.normalize();

			outGoingRayDir = Normal * DiffuseDir[0] + vertical * DiffuseDir[1] + lastOne * DiffuseDir[2];
			outGoingRayDir.normalize();



			// Here should have some attenuation;
			pathThroughput /= PI;

		}

		// Here, to Terminate the ray
		if (bounces > 3) {
			float continueProbability = min(.5f, Luna.dot(pathThroughput));
            if (RandomFloat() > continueProbability)
                break;
            //pathThroughput /= continueProbability;
        }
       
	if(!Traceresult.IntersectOrNot) break;   // If miss, just break.
	}

	return pathTrace_result;
}


void pathRender(void)
{
	int i, j, rayPerPixal;
	for (j = 0; j < IMAGE_HEIGHT; j++) 
	{
		for (i = 0; i < IMAGE_WIDTH; i++) 
		{


			if(i==25 && j==47)
				int kkkkkkk=333333;


			float Ir, Ig, Ib, Iracu, Igacu, Ibacu;
			Ir = Ig = Ib = Iracu = Igacu = Ibacu = 0;


			float sx = (i + 1.f/2) * (1.f / IMAGE_WIDTH);
			float sy = (j + 1.f/2) * (1.f / IMAGE_HEIGHT);

			Vector3f P;
			P = M + (2 * sx - 1) * X + (2 * sy - 1) * Y;

			Vector3f RayDir;
			RayDir = P - E;
			RayDir.normalize();



			int adaptiveSap = PER_PIXAL;
			int total = PER_PIXAL;

			for(rayPerPixal=0; rayPerPixal<adaptiveSap && adaptiveSap<MAX_SAMPLE; rayPerPixal++)
			{
				Vector3f res = PathTrace(RayDir,E);

				


				if(i>=1 && j>=1 && ((theImage(i-1,IMAGE_HEIGHT-j,0) - Ir > DISTANCE) 
					|| (theImage(i-1,IMAGE_HEIGHT-j,1) - Ig > DISTANCE) 
					|| (theImage(i-1,IMAGE_HEIGHT-j,2) - Ib > DISTANCE) || (theImage(i,IMAGE_HEIGHT-j,0) - Ir > DISTANCE) 
					|| (theImage(i,IMAGE_HEIGHT-j,1) - Ig > DISTANCE) 
					|| (theImage(i,IMAGE_HEIGHT-j,2) - Ib > DISTANCE) || (theImage(i-1,IMAGE_HEIGHT-j-1,0) - Ir > DISTANCE) 
					|| (theImage(i-1,IMAGE_HEIGHT-j-1,1) - Ig > DISTANCE) 
					|| (theImage(i-1,IMAGE_HEIGHT-j-1,2) - Ib > DISTANCE) ) )
					adaptiveSap += 1;

					




				Iracu += res[0];
				Ir = Iracu/adaptiveSap;

				Igacu += res[1];
				Ig = Igacu/adaptiveSap;

				Ibacu += res[2];
				Ib = Ibacu/adaptiveSap;

				//std::cout<<adaptiveSap<<std::endl;

			}






			//for(rayPerPixal=0; rayPerPixal<PER_PIXAL; rayPerPixal++)
			//{
			//	Vector3f res = PathTrace(RayDir,E);


			//	Ir += res[0]/PER_PIXAL;
			//	Ig += res[1]/PER_PIXAL;
			//	Ib += res[2]/PER_PIXAL;

			//	//std::cout<<adaptiveSap<<std::endl;

			//}



				Ir = Ir * 255;
				Ig = Ig * 255;
				Ib = Ib * 255;

				//std::cout << i << " " << j << " " << Ir << " " << Ig <<" " << Ib << std::endl;
				if (Ir > 0) { if (Ir <= 255) {theImage(i,IMAGE_HEIGHT-j-1,0) = Ir;} else {theImage(i,IMAGE_HEIGHT-j-1,0) = 255;}}
				if (Ig > 0) { if (Ig <= 255) {theImage(i,IMAGE_HEIGHT-j-1,1) = Ig;} else {theImage(i,IMAGE_HEIGHT-j-1,1) = 255;}}
				if (Ib > 0) { if (Ib <= 255) {theImage(i,IMAGE_HEIGHT-j-1,2) = Ib;} else {theImage(i,IMAGE_HEIGHT-j-1,2) = 255;}}
		}
	}
	return;
}
	




void render(void) {
	int i, j;
	float distance;
	
	for (j = 0; j < IMAGE_HEIGHT; j++) 
	{
		for (i = 0; i < IMAGE_WIDTH; i++) 
		{
			float sx = (i + 1.f/2) * (1.f / IMAGE_WIDTH);
			float sy = (j + 1.f/2) * (1.f / IMAGE_HEIGHT);

			Vector3f P;
			P = M + (2 * sx - 1) * X + (2 * sy - 1) * Y;

			Vector3f RayDir;
			RayDir = P - E;
			RayDir.normalize();


			if(i==75 && j ==133)
				int kkkk=3;




			TraceResult result;
			result = RayTrace_BVH(RayDir,E);

			if (result.IntersectOrNot)
			{
				//Shadding
				Vector3f Normal = result.Normal;
				float Ka,Kd,Ks,Ar,Ag,Ab,Dr,Dg,Db,Sr,Sg,Sb,kt;
				// Self Definition
				Ka = 1;
				Ks = 0.3;
				Kd = 1;

				// Load the Material Data
				float q = result.Material->shininess*128;

				kt = result.Material->ktran; //Transparancy

				Ar = result.Material->ambColor[0];
				Ag = result.Material->ambColor[1];
				Ab = result.Material->ambColor[2];

				Dr = result.Material->diffColor[0];
				Dg = result.Material->diffColor[1];
				Db = result.Material->diffColor[2];

				Sr = result.Material->specColor[0];
				Sg = result.Material->specColor[1];
				Sb = result.Material->specColor[2];

				delete[] result.Material;
				//Loop the light
				float DirectIllr,DirectIllg,DirectIllb;
				DirectIllr = DirectIllg = DirectIllb = 0;

				LightIO * FirstLight = scene->lights;
				Vector3f R;

				Vector3f view = E-result.Position;
				view.normalize();

				while ( scene->lights != NULL)
				{
					Vector3f LightPosition;
					Vector3f LightDir;
					float t;
					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						LightDir[0] = -scene->lights->direction[0];
						LightDir[1] = -scene->lights->direction[1];
						LightDir[2] = -scene->lights->direction[2];
					}
					
					else if( scene->lights->type == POINT_LIGHT )
					{
						LightPosition[0] = scene->lights->position[0];
						LightPosition[1] = scene->lights->position[1];
						LightPosition[2] = scene->lights->position[2];

						LightDir = LightPosition - result.Position;
					}

					LightDir.normalize();


					float Lr = scene->lights->color[0];
					float Lg = scene->lights->color[1];
					float Lb = scene->lights->color[2];


					R = 2 * Normal.dot(LightDir) * Normal - LightDir;
					R.normalize();
					Vector3f shadDir = LightDir;

					TraceResult LightResult = RayTrace_BVH(shadDir,result.Position + 0.001 * shadDir);
					float fattj;
					float S=1;
					if (scene->lights->type == POINT_LIGHT)
					{
						float x1,x2,x3;
						x1 = LightPosition[0] - result.Position[0];
						x2 = LightPosition[1] - result.Position[1];
						x3 = LightPosition[2] - result.Position[2];
						t = sqrt(x1 * x1 + x2 * x2 + x3 * x3);

						if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)
						{
							S = 0;
						}
						else
						{
							S = 1;
						}
						fattj = min(1, 1/(0.25 + 0.1 * t + 0.01 * pow(t,2)));
					}

					if (scene->lights->type == DIRECTIONAL_LIGHT)
					{
						if(LightResult.IntersectOrNot && LightResult.Min >= 0)
						{
							S = 0;
						}
						else
						{
							S = 1;
						}
						fattj = 1;
					}
					
					float kkk = (Normal.dot(LightDir));
					kkk = (kkk<0) ? 0:kkk;

					// Why I set the q to be 0.2 will cause bugs? The S is 0, why can not the plus result be 0?????????????????????????????????????????
					//float tst = R.dot(V);
					//std::cout << tst  << std::endl;

					DirectIllr = (S * Lr* fattj * ((1-kt) * kkk * Dr + Sr*pow(clamp(R.dot(view)),q))) + DirectIllr;
					DirectIllg = (S * Lg* fattj * ((1-kt) * kkk * Dg + Sg*pow(clamp(R.dot(view)),q))) + DirectIllg;
					DirectIllb = (S * Lb* fattj * ((1-kt) * kkk * Db + Sb*pow(clamp(R.dot(view)),q))) + DirectIllb;

					scene->lights = scene->lights->next;
				}

				scene->lights = FirstLight;
				traceTime = 0;

				Vector3f RView = 2 * Normal.dot(view) * Normal - view;
				RView.normalize();

				
				Vector3f RefractionDir_next = getRefractionDir(result.Normal,-view,1,1.5);// Here change the RayDir to -view
				Vector3f reflectClr = Reflection(Sr,Sg,Sb, RView, result.Position + MOVE_FACTOR * RView);
				

				Vector3f RefractionRes = Refraction(kt, RefractionDir_next, result.Position + MOVE_FACTOR * RefractionDir_next);



				//Vector3f pathTraceRes = PathTrace(Sr,Sg,Sb, RView, result.Position);

				//Vector3f zero(0,0,0);

				//pathTraceRes = zero;

				float Ir = (((1-kt) * Ar * Dr + DirectIllr) + Sr * reflectClr[0] + kt * RefractionRes[0]/* + pathTraceRes[0]*/) * 255;
				float Ig = (((1-kt) * Ag * Dg + DirectIllg) + Sg * reflectClr[1] + kt * RefractionRes[1]/* + pathTraceRes[1]*/) * 255;
				float Ib = (((1-kt) * Ab * Db + DirectIllb) + Sb * reflectClr[2] + kt * RefractionRes[2]/* + pathTraceRes[2]*/) * 255;

				//std::cout << i << " " << j << " " << Ir << " " << Ig <<" " << Ib << std::endl;
				if (Ir > 0) { if (Ir <= 255) {theImage(i,IMAGE_HEIGHT-j-1,0) = Ir;} else {theImage(i,IMAGE_HEIGHT-j-1,0) = 255;}}
				if (Ig > 0) { if (Ig <= 255) {theImage(i,IMAGE_HEIGHT-j-1,1) = Ig;} else {theImage(i,IMAGE_HEIGHT-j-1,1) = 255;}}
				if (Ib > 0) { if (Ib <= 255) {theImage(i,IMAGE_HEIGHT-j-1,2) = Ib;} else {theImage(i,IMAGE_HEIGHT-j-1,2) = 255;}}
			}
		}
	}
	return;
}



void display(void)
{
	theImage.save_bmp("output.bmp"); // write it
	theImage.display();
	//CImgDisplay main_disp(theImage); // display it
	//	CImgDisplay main_disp(earth); // display it
		
		std::cin.ignore();
}



int main(int argc, char *argv[]) {
	Timer total_timer;
	total_timer.startTimer();


	//objLoader *objData = new objLoader();
	//objData->load("cornell_box.obj");
	//hdrImage->loadPfm("4.pfm");




	loadScene("box_light_2.ascii");
	// Question: How to definie the per vertex normal? what is the meaning of that?
	loadScene2Previews(scene);

	BVH_TreeNode * root;
	std::cout<<NumberOfPrimes<<std::endl;
	root = recursiveBuild(0,NumberOfPrimes);   // Why here we do not need to minus 1!?????????????????????????????????????????????????????

	std::cout<<totalBVH_TreeNode<<","<<std::endl;
	//nodes = AllocAligned<LinearBVHNode>(NumberOfPrimes);

	nodes = new LinearBVHNode[totalBVH_TreeNode];

    int offset = 0;
    flattenBVHTree(root, &offset);

	std::cout<<offset<<std::endl;


	/* write your ray tracer here */
	//render();

	pathRender();

	display();

	std::cout<<'\a';
	/* cleanup */
	if (scene != NULL) {
		deleteScene(scene);
	}

	total_timer.stopTimer();
	fprintf(stderr, "Total time: %.5lf secs\n\n", total_timer.getTime());
	
	std::cout<<total_timer.getTime()<<std::endl;

	std::cin.ignore();
	return 1;
}


boolean Ray_Box_Interset(BBox box, Vector3f RayDir, Vector3f RayOri)
{
	
	Vector3f invDir(1.f / RayDir[0], 1.f / RayDir[1], 1.f / RayDir[2]);
    int dirIsNeg[3] = { invDir[0] < 0, invDir[1] < 0, invDir[2] < 0 };


	Vector3f MinMax[2];
	MinMax[0] = box.pMin;
	MinMax[1] = box.pMax;



	float tmin =  (MinMax[  dirIsNeg[0]][0] - RayOri[0]) * invDir[0];
    float tmax =  (MinMax[1-dirIsNeg[0]][0] - RayOri[0]) * invDir[0];
    float tymin = (MinMax[  dirIsNeg[1]][1] - RayOri[1]) * invDir[1];
    float tymax = (MinMax[1-dirIsNeg[1]][1] - RayOri[1]) * invDir[1];
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    // Check for ray intersection against $z$ slab
    float tzmin = (MinMax[  dirIsNeg[2]][2] - RayOri[2]) * invDir[2];
    float tzmax = (MinMax[1-dirIsNeg[2]][2] - RayOri[2]) * invDir[2];
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
	return true;
}

Vector3f randomDir()
{
	Vector3f result;
	result[0] = RandomFloat();
	result[1] = 2 * RandomFloat() - 1;
	result[2] = 2 * RandomFloat() - 1;

	result.normalize();
	return result;
}

Vector3f randomAllDir()
{
	Vector3f result;
	result[0] = 2 * RandomFloat() - 1;
	result[1] = 2 * RandomFloat() - 1;
	result[2] = 2 * RandomFloat() - 1;

	result.normalize();
	return result;
}


Vector3f AbsorbedIllumination(Ray ray)
{
	Vector3f result;
	Vector3f ori = ray.Ori;
	Vector3f des = ray.Destination;

	float dist_sq = (ori[0] - des[0]) * (ori[0] - des[0]) + (ori[1] - des[1]) * (ori[1] - des[1]) + (ori[2] - des[2]) * (ori[2] - des[2]);
	float dist = pow(dist_sq, 0.5);
	float absorption=exp(-sigma_t * dist);

	result = ray.Illumination * absorption;

	return result;
}


float getDist(Vector3f ori, Vector3f des)
{
	float dist_sq = (ori[0] - des[0]) * (ori[0] - des[0]) + (ori[1] - des[1]) * (ori[1] - des[1]) + (ori[2] - des[2]) * (ori[2] - des[2]);
	float dist = pow(dist_sq, 0.5);
	
	return dist;
}


Vector3f ViewResult(Ray ray, Vector3f LightSoursePosition, Vector3f LightColor)
{
	Vector3f rgb_result;
	Vector3f ori = ray.Ori;
	Vector3f des = ray.Destination;
	ray.Dir.normalize();

	float dist = getDist(ori,des);



	Vector3f Light_Integ_result(0,0,0);

	
	int cuttimes;

	//if(dist<1) cuttimes = INTEGRATE;
	//else cuttimes = dist/1 + INTEGRATE;

	//if(cuttimes > 20) cuttimes = 20;
	// Here, Use the Dynamic array
	cuttimes = ceil(dist) * 10;
	if (dist > 100) cuttimes = 150;

	Vector3f* positions = new Vector3f[cuttimes];

	

	for(int i=0; i<cuttimes; i++)
	{
		Vector3f currentP = ori + float(i+1)/(cuttimes) * dist * ray.Dir;
		positions[i] = currentP; 
	}

	for(int i=0; i<cuttimes; i++)
	{
		Vector3f LightDir = LightSoursePosition - positions[i];

		Vector3f shadDir = LightDir;
		shadDir.normalize();

		float dist_temp = getDist(LightSoursePosition,positions[i]);
		float dist_position2des = getDist(des,positions[i]);


		TraceResult LightResult = RayTrace(shadDir,positions[i] + 0.001 * shadDir);
			float fattj;
			float S=1;
			float t = 0;
			t = dist_temp;

			if(LightResult.IntersectOrNot == 1 && LightResult.Min >= 0 && LightResult.Min <= t)
				S = 0;
			else 
				S = 1;

			Vector3f lightDirection_(0,1,0);
			lightDirection_.normalize();		

			// NEW/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			float tempcos = shadDir.dot(lightDirection_);
			if((tempcos>=0.8) && tempcos<1.1)
				S+=1;
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			if((tempcos<=0.15) || tempcos>1.1)
				S=0;
			



			Vector3f trueLightDir = positions[i] - LightSoursePosition;
			trueLightDir.normalize();


			float kkk = abs(trueLightDir.dot(ray.Dir));


			float plusportion = FOG * kkk *  S * exp(-sigma_t * 5 * dist_temp) *  exp(-sigma_t * 5 * dist_position2des) / cuttimes;

			//if(plusportion>0)
				Light_Integ_result += LightColor *  plusportion;

			delete LightResult.Material;
	}


	delete positions;

	float totalabsorb = exp(-sigma_t * 5 * getDist(ori,des));

	ray.Illumination *= totalabsorb;
	rgb_result = ray.Illumination + Light_Integ_result;

	return rgb_result;
}