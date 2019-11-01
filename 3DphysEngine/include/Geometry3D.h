#pragma once

#include "Transformation.h"
#include "AABB.h"
#include "Sphere.h"
#include "Plain.h"
#include <vector>
#include <iostream>
#include "Clipper.h"
using namespace ClipperLib;
struct Line {
public:
	glm::vec3 start;
	glm::vec3 end;
	Line(const glm::vec3& m_start, const glm::vec3& m_end)
		:start(m_start), end(m_end)
	{}
};
struct Spher {
public:
	float radius;
	glm::vec3 center;
	Spher(const float m_radius, const glm::vec3& m_center)
		:radius(m_radius), center(m_center)
	{}
};

class Geometry3D
{
public:
	static bool broadPhase(Spher s1, Spher s2)
	{
		float r = s1.radius + s2.radius;
		glm::vec3 dist = s2.center - s1.center;
		float LengDist = glm::length(dist);
		if (LengDist - r > 0 || LengDist == 0.0f)
		{
			return false;
		}
		return true;
	}
	static std::vector<glm::vec2> intersectionOfPolyhedrons(std::vector<glm::vec2>& p1, std::vector<glm::vec2>& p2)
	{
		float e = 100000000.0f;
		std::vector<glm::vec2> result;
		glm::vec2 temp = glm::vec2(0.0f);
	/*	if (p1.size() == 4 && p2.size() == 4)
			int kek = 4;
		for (int i = 0; i < p1.size(); ++i)
		{
			if (abs(p1[i].x) > temp.x && abs(p1[i].y) < temp.y)
				temp = p1[i];
		}
		for (int i = 0; i < p1.size(); ++i)
		{
			if (abs(p1[i].x) > temp.x && abs(p1[i].y) < temp.y)
				temp = p1[i];
		}*/

  		if (p1.size() == 4)
		{
			glm::vec2 temp = p1[1];
			p1[1] = p1[2];
			p1[2] = temp;

			glm::vec2 temp1 = p1[2];
			p1[2] = p1[3];
			p1[3] = temp1;
		}
		if (p2.size() == 4)
		{
			glm::vec2 temp = p2[1];
			p2[1] = p2[2];
			p2[2] = temp;

		}
		if (p1.size() == 4 && p2.size() == 1)
		{
			result.push_back(p2[0]);
		}
		if (p1.size() == 1 && p2.size() == 4)
		{
			result.push_back(p1[0]);
		}
		if (p1.size() == 1 && p2.size() == 2)
		{
			result.push_back(p1[0]);
		}
		if (p1.size() == 2 && p2.size() == 1)
		{
			result.push_back(p2[0]);
		}
		if (p1.size() == 1 && p2.size() == 1)
		{
			result.push_back(p1[0]);
		}
		if (p1.size() == 4 && p2.size() == 4)
		{
			Paths subj(1), clip(1);
 			Paths solution;
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * e, p1[i].y * e));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * e , p2[i].y * e));
			}
			Clipper c;
			c.AddPaths(subj, ptSubject, true);
			c.AddPaths(clip, ptClip, true);
			c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
			if (!solution.empty())
			{
				for (int i = 0; i < solution[0].size(); ++i)
				{
					result.push_back(glm::vec2(solution[0][i].X / e, solution[0][i].Y / e));
				}
			}
			//std::cout << "1" << " " << result.size() << std::endl;
		}
		if (p1.size() == 4 && p2.size() == 2)
		{
			Paths subj(1);
			Paths clip(1);
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * e, p1[i].y * e));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * e, p2[i].y * e));
			}
			Clipper c;
			PolyTree solution;
			c.AddPaths(clip, ptSubject, false);
			c.AddPaths(subj, ptClip, true);
			c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
			Paths res; 
			PolyTreeToPaths(solution, res);
			if (!res.empty())
			{
				for (int i = 0; i < res[0].size(); ++i)
				{
					result.push_back(glm::vec2(res[0][i].X / e, res[0][i].Y / e));
				}
			}
			//std::cout << "2" << " " << result.size() << std::endl;
		}
		if (p1.size() == 2 && p2.size() == 4)
		{
			Paths subj(1), clip(1);
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * e, p1[i].y * e));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * e, p2[i].y * e));
			}
			Clipper c;
			PolyTree solution;
			c.AddPaths(subj, ptSubject, false);
			c.AddPaths(clip, ptClip, true);
			c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);

			Paths res;
			PolyTreeToPaths(solution, res);
			if (!res.empty())
			{
				for (int i = 0; i < res[0].size(); ++i)
				{
					result.push_back(glm::vec2(res[0][i].X / e, res[0][i].Y / e));
				}
			}
			//std::cout << "3" << " " << result.size() << std::endl;
		}
		if (p1.size() == 2 && p2.size() == 2)
		{
   			glm::vec2 a = p1[0];
			glm::vec2 b = p1[1];
			glm::vec2 c = p2[0];
			glm::vec2 d = p2[1];
			glm::vec2 T;
			
			T.x = -((a.x * b.y - b.x * a.y) * (d.x - c.x) - (c.x * d.y - d.x * c.y) * (b.x - a.x)) / ((a.y - b.y) * (d.x - c.x) - (c.y - d.y) * (b.x - a.x));
			T.y = ((c.y - d.y) * (-T.x) - (c.x * d.y - d.x * c.y)) / (d.x - c.x);
			result.push_back(std::move(T));		
			//std::cout << "4" << " " << result.size() << std::endl;
		}		
		return std::move(result);
	}
};