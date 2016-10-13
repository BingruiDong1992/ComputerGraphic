#include <windows.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include "scene_io.h"
#include "Timer.h"
#include "CImg.h"
#include "Eigen\Eigen"
#include "Eigen\Dense" 

#define PI 3.14159265
using namespace cimg_library;
using namespace Eigen;
using Eigen::MatrixXd;

float RaySphereIntersection (Vector3f SphereCenter, float Radius, Vector3f RayDir, Vector3f RayOri)
{
	float d = 0;

	float define = (pow((RayDir.dot(RayOri - SphereCenter)),2) - (RayOri - SphereCenter).dot(RayOri - SphereCenter) + Radius*Radius);
	if(define > 0)
	{
		float d = -(RayDir.dot(RayOri - SphereCenter)) - sqrt(define);
		if (d<0)
		{
			d = -(RayDir.dot(RayOri - SphereCenter)) + sqrt(define);
		}
		if (d>0) {return d;}
		
		else{ return 0;}
	}
	else
	{
		return 0;
	}
}

Vector3f RayTriangleIntersection (Vector3f V0, Vector3f V1, Vector3f V2, Vector3f RayDir, Vector3f RayOri)
{
	Vector3f E1 = V1 - V0;
	Vector3f E2 = V2 - V0;
	Vector3f T = RayOri - V0;
	Vector3f P = RayDir.cross(E2);
	Vector3f Q = T.cross(E1);

	float t = Q.dot(E2) / P.dot(E1);
	float u = P.dot(T) / P.dot(E1);
	float v = Q.dot(RayDir) / P.dot(E1);

	Vector3f result;
	result[0] = t;
	result[1] = u;
	result[2] = v;
	return result;
}

bool EncounterTriOrNot (Vector3f LigPosition, Vector3f LigDir, Vector3f V0, Vector3f V1, Vector3f V2)
{

	return TRUE;

}


Vector3f getRefractionDir (Vector3f Normal, Vector3f RayDir, float n1, float n2 )
{
	RayDir = -RayDir;
	float define = Normal.dot(RayDir);
	float theta1 = std::acos(std::abs(Normal.dot(RayDir)));

	if (define<0)
	{
		//Not sure???????????????????????????????????????????????????????????????????????????????????????????????????
		Normal = - Normal;
		float theta2 = std::asin (sin(theta1) * 1.5f / 1.f);
		Vector3f Q = cos(theta1) * Normal;
		Vector3f M = (sin(theta2)/sin(theta1)) * (Q - RayDir);
		Vector3f P = -cos(theta2) * Normal;
		Vector3f T = M + P;
		T.normalize();
		return T;
		


	}
	if (define >= 0)
	{
		float theta2 = std::asin (sin(theta1) * 1.f / 1.5f);
		Vector3f Q = cos(theta1) * Normal;
		Vector3f M = (sin(theta2)/sin(theta1)) * (Q - RayDir);
		Vector3f P = -cos(theta2) * Normal;
		Vector3f T = M + P;
		T.normalize();
		return T;
	}
}

float clamp(float x)
{
	if(x>0)
	{return x;}
	else
	{return 0;}
}

Vector3f Normalize_color(float x, float y, float z)
{
	float max = max(x,y,z);
	Vector3f result;
	result[0] = x/max;
	result[1] = y/max;
	result[2] = z/max;
	return result;
}


float RandomFloat() 
{
    
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

	return r;
}

Vector3f getVertical(Vector3f A)
{
	Vector3f result;
	Vector3f C;
	if (A[1] != 0 || A[2] != 0)
	{
		C[0] = 1;
		C[1] = 0;
		C[2] = 0;
	}
	else
	{
		C[0] = 0;
		C[1] = 1;
		C[2] = 0;
	}

	Vector3f B = A.cross(C);

	return B;
}





