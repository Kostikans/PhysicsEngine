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
	{
	}
};


class Geometry3D
{
public:
	static std::vector<glm::vec2> intersectionOfPolyhedrons(std::vector<glm::vec2>& p1, std::vector<glm::vec2>& p2)
	{

		std::vector<glm::vec2> result;
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
			result.push_back(p2[0]);
		}
		if (p1.size() == 4 && p2.size() == 4)
		{
			Paths subj(1), clip(1);
 			Paths solution;
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * 100000.0f, p1[i].y * 100000.0f));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * 100000.0f, p2[i].y * 100000.0f));
			}
			Clipper c;
			c.AddPaths(subj, ptSubject, true);
			c.AddPaths(clip, ptClip, true);
			c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);

			for (int i = 0; i < solution[0].size(); ++i)
			{
				result.push_back(glm::vec2(solution[0][i].X / 100000.0f, solution[0][i].Y / 100000.0f));
			}
		}
		if (p1.size() == 4 && p2.size() == 2)
		{
			Paths subj(1);
			Paths clip(1);
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * 100000.0f, p1[i].y * 100000.0f));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * 100000.0f, p2[i].y * 100000.0f));
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
					result.push_back(glm::vec2(res[0][i].X / 100000.0f, res[0][i].Y / 100000.0f));
				}
			}
		}
		if (p1.size() == 2 && p2.size() == 4)
		{
			Paths subj(1), clip(1);
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * 100000.0f, p1[i].y * 100000.0f));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * 100000.0f, p2[i].y * 100000.0f));
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
					result.push_back(glm::vec2(res[0][i].X / 100000.0f, res[0][i].Y / 100000.0f));
				}
			}
		}
		if (p1.size() == 2 && p2.size() == 2)
		{
 			Paths subj(1), clip(1);
			for (int i = 0; i < p1.size(); ++i)
			{
				subj[0].push_back(IntPoint(p1[i].x * 100000.0f, p1[i].y * 100000.0f));
			}
			for (int i = 0; i < p2.size(); ++i)
			{
				clip[0].push_back(IntPoint(p2[i].x * 100000.0f, p2[i].y * 100000.0f));
			}
			Clipper c;
			PolyTree solution;
			c.AddPaths(subj, ptSubject, false);
			c.AddPaths(clip, ptSubject, false);
			c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);

			Paths res;
			PolyTreeToPaths(solution, res);
			if (!res.empty())
			{
				for (int i = 0; i < res[0].size(); ++i)
				{
					result.push_back(glm::vec2(res[0][i].X / 100000.0f, res[0][i].Y / 100000.0f));
				}
			}
		}
		
		
		return result;
	}
};