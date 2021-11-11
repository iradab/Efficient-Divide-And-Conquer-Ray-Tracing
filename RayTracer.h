
#pragma once

#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

#include "Vec3.h"
#include "Image.h"
#include "Camera.h"
#include "Scene.h"
#include "EDACRT.h"
#include "AABB.h"
#include <chrono>

using namespace std;

class RayTracer {
public:
	RayTracer() {}
	virtual ~RayTracer() {}
	inline float floatRandom(const float min, const float max) {
		float LO = min;
		float HI = max;
		return LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
	}

	inline bool rayTrace(const Ray& ray, const Scene& scene, size_t& meshIndex, size_t& triangleIndex, float& u, float& v, float& d) {
		const auto& meshes = scene.meshes();
		float closest = std::numeric_limits<float>::max();
		bool intersectionFound = false;
		for (size_t mIndex = 0; mIndex < meshes.size(); mIndex++) {
			const auto& P = meshes[mIndex].vertexPositions();
			const auto& T = meshes[mIndex].indexedTriangles();
			for (size_t tIndex = 0; tIndex < T.size(); tIndex++) {
				const Vec3i& triangle = T[tIndex];
				float ut, vt, dt;
				if (ray.triangleIntersect(P[triangle[0]], P[triangle[1]], P[triangle[2]], ut, vt, dt) == true) {
					if (dt > 0.f && dt < closest) {
						intersectionFound = true;
						closest = dt;
						meshIndex = mIndex;
						triangleIndex = tIndex;
						u = ut;
						v = vt;
						d = dt;
					}
				}
			}
		}
		return intersectionFound;
	}

	inline Vec3f shade(const Scene& scene, size_t meshIndex, Vec3i triangleIndex, float u, float v) {
		const auto& mesh = scene.meshes()[meshIndex];
		const auto& P = mesh.vertexPositions();
		const auto& N = mesh.vertexNormals();
		const Vec3i& triangle = triangleIndex;
		Vec3f hitNormal = normalize((1.f - u - v) * N[triangle[0]] + u * N[triangle[1]] + v * N[triangle[2]]);

		Vec3f hitPos = (1.f - u - v) * P[triangle[0]] + u * P[triangle[1]] + v * P[triangle[2]];
		Vec3f wi = normalize(scene.light_sources()[0].source_position() - hitPos); 
		Vec3f colorResponse = Vec3f(0, 0, 0);
		float u2, v2, d2;
		size_t meshIndex2;
		size_t triangleIndex2;
		for (int i = 0; i < 16; i++) {
			Vec3f lightPos = scene.light_sources()[0].source_position() + Vec3f(floatRandom(0, 0.2), floatRandom(0, 0.2), floatRandom(0, 0.2));
			Ray ray(hitPos + 0.001f * ((lightPos - hitPos) / normalize(lightPos - hitPos)), normalize(lightPos - hitPos));
			bool intersectionFound = rayTrace(ray, scene, meshIndex2, triangleIndex2, u2, v2, d2);
			if (intersectionFound == false)
				colorResponse += scene.meshes()[meshIndex].getMaterial().evaluateColorResponse(hitNormal, wi);
		}
		return colorResponse / 16;
	}

	AABB createAABBforTriangles(std::vector<Vec3i> p, Scene scene) {
		const auto& meshes = scene.meshes();
		const auto& P = meshes[0].vertexPositions();
		AABB result;
		float xmin = 1000, xmax = -1000;
		float ymin = 1000, ymax = -1000;
		float zmin = 1000, zmax = -1000;

		for (int i = 0; i < p.size(); i++) {
			for (int j = 0; j < 3; j++) {
				if (P[p[i][j]][0] < xmin)
					xmin = P[p[i][j]][0];
				if (P[p[i][j]][1] < ymin)
					ymin = P[p[i][j]][1];
				if (P[p[i][j]][2] < zmin)
					zmin = P[p[i][j]][2];
				if (P[p[i][j]][0] > xmax)
					xmax = P[p[i][j]][0];
				if (P[p[i][j]][1] > ymax)
					ymax = P[p[i][j]][1];
				if (P[p[i][j]][2] > zmax)
					zmax = P[p[i][j]][2];
			}
		}

		result.m_start = Vec3f(xmin, ymin, zmin);
		result.m_end = Vec3f(xmax, ymax, zmax);
		return result;
	}
	inline void render(const Scene& scene, Image& image) {
		size_t w = image.width();
		size_t h = image.height();
		int nRPP = 4; // number of rays per pixel
		const Camera& camera = scene.camera();
		std::vector<Ray> Rays;
		const int nb = w * h * nRPP;
		std::vector<int> xs, ys;
		std::vector<int> rayIndices(nb);
		std::vector<Ray> raysR(nb);
		std::vector<Vec3i> trianglesR(nb);

		std::vector<float> uR(nb), vR(nb), dR(nb, 10000);
		std::vector<bool> intersectionsFoundR(nb);
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				for (int i = 0; i < nRPP; i++) {
					Ray ray = camera.rayAt(floatRandom(float(x) / w, float(x + 1) / w), floatRandom(1.f - float(y) / h, 1.f - float(y + 1) / h));
					Rays.push_back(ray);
				}
			}
		}
		for (int i = 0; i < nb; i++) {
			rayIndices[i] = i;
			trianglesR[i] = Vec3i(0, 0, 0);
			uR[i] = 0;
			vR[i] = 0;
			intersectionsFoundR[i] = false;
		}

		const auto& meshes = scene.meshes();
		AABB BV = createAABBforTriangles(meshes[0].indexedTriangles(), scene);
		EDACRT E = EDACRT();
		std::vector<Vec3i> triangles = meshes[0].indexedTriangles();
		std::vector<int> xR, yR;
		std::cout << "Number of rays: " << Rays.size() << " .Number of triangles: " << triangles.size() << std::endl;
		E.DACRT(Rays, rayIndices, BV, triangles, scene, trianglesR, uR, vR, dR, intersectionsFoundR);
		std::cout << "EDACRT is done" << std::endl;

		for (int y = 0; y < h; y++) {
#pragma omp parallel for
			for (int x = 0; x < w; x++) {
				bool anyIntersection = false;
				Vec3f colorResponse = Vec3f(0, 0, 0);
				for (int i = 0; i < nRPP; i++) {
					int k = nRPP * (w * y + x) + i;
					if (intersectionsFoundR[k] == true) {
						anyIntersection = true;
						colorResponse += shade(scene, 0, trianglesR[k], uR[k], vR[k]);
					}
				}
				if (anyIntersection)
					image(x, y) = colorResponse / nRPP;
			}
		}
	}
};