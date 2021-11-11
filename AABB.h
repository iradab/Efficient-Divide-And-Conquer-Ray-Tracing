#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <exception>

#include "Vec3.h"
#include "Ray.h"

class AABB {
public:
	inline AABB() {}

	inline AABB(Vec3f& start, Vec3f& end) :
		m_start(start), m_end(end) {} 

	virtual ~AABB() {}

	// Ray box intersection function reference to online lesson on ray box intersection test based on An Efficient and Robust Ray–Box Intersection Algorithm. Amy Williams et al. 2004. 
	bool rayIntersection(const Ray& ray, float& distance) {
		if (m_start == Vec3f(0, 0, 0) && m_end == Vec3f(0, 0, 0)) {
			distance = 0;
			return false;
		}
		float tmina[3] = {0}, tmaxa[3] = {0};
		for (int i = 0; i < 3; i++) {
			tmina[i] = (m_start[i]-ray.origin()[i])/ray.direction()[i];
			tmaxa[i] = (m_end[i]-ray.origin()[i])/ray.direction()[i];

			if (tmina[i] > tmaxa[i]) std::swap(tmina[i], tmaxa[i]);
			if (i != 0) {
				if ((tmina[0] > tmaxa[i]) || (tmina[i] > tmaxa[0]))
					return false;
				if (tmina[i] > tmina[0])
					tmina[0] = tmina[i];
				if (tmaxa[i] < tmaxa[0])
					tmaxa[0] = tmaxa[i];
			}
		}
		distance = tmina[0];
		return true;
	}

	Vec3f center() {
		Vec3f centerP;
		for (int i = 0; i < 3; i++) {
			centerP[i] = m_start[i] + ((m_end[i] - m_start[i]) / 2);
		}
		return centerP;
	}

	Vec3f m_start;
	Vec3f m_end;
};
