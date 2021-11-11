#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <exception>

#include "Vec3.h"
#include "Ray.h"
#include "AABB.h"
#include "Scene.h"
#include <algorithm>

class EDACRT {
public:
    inline EDACRT() {}

    virtual ~EDACRT() {}

    inline void naiveRayIntersection(std::vector<Ray>& rays, std::vector<int> rayIndices, std::vector<Vec3i>& triangles, const Scene& scene, std::vector<Vec3i>& triangleIndicesR,
        std::vector<float>& u, std::vector<float>& v, std::vector<float>& d, std::vector<bool>& intersectionsFound) {
        const auto& meshes = scene.meshes();
        std::vector<int> rayIndexes;
        for (size_t rIndex = 0; rIndex < rays.size(); rIndex++) {
            float closest = std::numeric_limits<float>::max();
            float intersectionFound = false;
            float ut, vt, dt;
            float uF, vF, dF;
            Vec3i triangleI;
            for (size_t mIndex = 0; mIndex < meshes.size(); mIndex++) {
                const auto& P = meshes[mIndex].vertexPositions();
                for (size_t tIndex = 0; tIndex < triangles.size(); tIndex++) {
                    const Vec3i& triangle = triangles[tIndex];
                    if (rays[rIndex].triangleIntersect(P[triangle[0]], P[triangle[1]], P[triangle[2]], ut, vt, dt) == true) {
                        if (dt > 0.f && dt < closest) {
                            intersectionFound = true;
                            closest = dt;
                            triangleI = triangle;
                            uF = ut;
                            vF = vt;
                            dF = dt;
                        }
                    }
                }
            }

            if (intersectionFound == true) {
                if (d[rayIndices[rIndex]] >= dF) {
                    intersectionsFound[rayIndices[rIndex]] = true;
                    triangleIndicesR[rayIndices[rIndex]] = triangleI;
                    u[rayIndices[rIndex]] = uF;
                    v[rayIndices[rIndex]] = vF;
                    d[rayIndices[rIndex]] = dF;
                }
            }
        }
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
        if (result.m_start == Vec3f(1000, 1000, 1000))
            result.m_start = Vec3f(0, 0, 0);
        if (result.m_end == Vec3f(-1000, -1000, -1000))
            result.m_end = Vec3f(0, 0, 0);
        return result;
    }
    Vec3f findCenterofTriangle(Vec3i& triangle, Scene& scene) {
        std::vector<Vec3i> triangles;
        triangles.push_back(triangle);
        AABB BBforTriangle = createAABBforTriangles(triangles, scene);
        return BBforTriangle.center();
    }

    bool belongsToBV(Vec3f& center, AABB B) {

        if (center[0] > B.m_start[0] && center[0] < B.m_end[0]
            && center[1] > B.m_start[1] && center[1] < B.m_end[1]
            && center[2] > B.m_start[2] && center[2] < B.m_end[2]) {
            return true;
        }
        else { return false; }
    }

    // fills the vectors Vl, Vr, Tl, Tr
    void calculate(std::vector<AABB> B, std::vector<Vec3i> triangles,
        std::vector<std::vector<Vec3i>>& Tl, std::vector<std::vector<Vec3i>>& Tr, std::vector<AABB>& Vl, std::vector<AABB>& Vr, Scene scene) {
        for (int k = 0; k < B.size(); k++) {
            std::vector<Vec3i> left, right;
            bool belongs;
            for (int i = 0; i < triangles.size(); i++) {
                Vec3f center = findCenterofTriangle(triangles[i], scene);
                belongs = belongsToBV(center, B[k]); // belongs to left
                if (belongs == true) {
                    left.push_back(triangles[i]);
                }
                else {
                    right.push_back(triangles[i]);
                }
            }
            Tl.push_back(left);
            Tr.push_back(right);

            Vl.push_back(createAABBforTriangles(Tl[k], scene));
            Vr.push_back(createAABBforTriangles(Tr[k], scene));
        }
    }
    void intersect(Ray ray, std::vector<AABB> Vl, std::vector<AABB> Vr, int cl[], int cr[], int nl[], int nr[], int K) {
        for (int j = 0; j < K; j++) {
            float dl = 10000, dr = 10000;
            float tn1, tn2;
            bool inters1 = false, inters2 = false;
            if (Vl[j].rayIntersection(ray, tn1)) {
                inters1 = true;
                cl[j]++; dl = tn1;
            }
            if (Vr[j].rayIntersection(ray, tn2)) {
                inters2 = true;
                cr[j]++; dr = tn2;
            }
            if (dl < dr)
                nl[j]++;
            else
                nr[j]++;

        }
    }

    std::vector<AABB> subdivide_V_into_K_bins(AABB V, int K) {
        float maxDist = 0;
        int divideI = -1; // axis which we divide
        for (int i = 0; i < 3; i++) {
            if ((V.m_end[i] - V.m_start[i]) > maxDist) {
                maxDist = V.m_end[i] - V.m_start[i];
                divideI = i;
            }
        }
        std::vector<AABB> B;
        for (int i = 0; i < K; i++) {
            AABB b;
            B.push_back(b);
            for (int j = 0; j < 3; j++) {
                B[i].m_start[j] = V.m_start[j];
                if (j == divideI) {
                    B[i].m_end[j] = ((i + 1) * maxDist) / (K + 1) + V.m_start[j];
                }
                else
                    B[i].m_end[j] = V.m_end[j];
            }
        }
        return B;
    }

    //a bounding volume V, a set of active rays R, and a set of triangles T 
    void DACRT(std::vector<Ray>& rays, std::vector<int> rayIndices, AABB V, std::vector<Vec3i>& triangles, Scene scene, std::vector<Vec3i>& trianglesR,
        std::vector<float>& uR, std::vector<float>& vR, std::vector<float>& dR, std::vector<bool>& intersectionsFoundR) {
        //naiveRayIntersection(rays, rayIndices, triangles, scene, trianglesR, uR, vR, dR, intersectionsFoundR);
        //std::cout << "Rays: " << rays.size() << "  " << triangles.size() << std::endl;
        
        const int K = 4;
        if (rays.size() < 50 || triangles.size() < 50) {            // check if it is leaf or not
            naiveRayIntersection(rays, rayIndices, triangles, scene, trianglesR, uR, vR, dR, intersectionsFoundR);
            return;
        }
        std::vector<AABB> B = subdivide_V_into_K_bins(V, K);
        std::vector<std::vector<Vec3i>> Tl, Tr;
        std::vector<AABB> Vl, Vr;
        calculate(B, triangles, Tl, Tr, Vl, Vr, scene);
        int cl[K] = { 0 }, cr[K] = { 0 }, nl[K] = { 0 }, nr[K] = { 0 };

        
        float Ns; 
        if (rays.size() > 1000) {
            std::vector<int> sampled_ray_indices;
            for (int i = 0; i < 100; i++) {
                sampled_ray_indices.push_back(rand() % rays.size());
            }
            for (int i = 0; i < 100; i++) {                               
                intersect(rays[sampled_ray_indices[i]], Vl, Vr, cl, cr, nl, nr, K);                    
            }
            Ns = 100;
        }
        else {
            Ns = rays.size();
            for (int i = 0; i < rays.size(); i++) {                               
                intersect(rays[i], Vl, Vr, cl, cr, nl, nr, K);                    
            }
        }

        // finding jmin, Cmin - partitioning
        float Cmin = 1000000;
        int jmin = 1;
        float Ct = 1, Ci = 1;
        float alphaL[K], alphaR[K];
        float C;
        for (int i = 0; i < K; i++) {
            alphaL[i] = cl[i] / Ns;
            alphaR[i] = cr[i] / Ns;
            C = alphaL[i] * Tl[i].size() + alphaR[i] * Tr[i].size(); 
            if (C <= Cmin) {
                jmin = i; Cmin = C;
            }

        }
        AABB V0, V1;
        float alpha0, alpha1;
        std::vector<Vec3i> T0, T1;

        // determining traversal order 
        
        if (nl[jmin] >= nr[jmin]) {
            V0 = Vl[jmin]; V1 = Vr[jmin];
            alpha0 = alphaL[jmin]; alpha1 = alphaR[jmin];
            T0 = Tl[jmin]; T1 = Tr[jmin];
        }
        else {
            V0 = Vr[jmin]; V1 = Vl[jmin];
            alpha0 = alphaR[jmin]; alpha1 = alphaL[jmin];
            T0 = Tr[jmin]; T1 = Tl[jmin];
        }

        std::vector <Ray> R0;

        std::vector<int> rayIndices2;      

        // ray filtering or skipping it 
        if (alpha0 > 0.5) { 
            DACRT(rays, rayIndices, V0, T0, scene, trianglesR, uR, vR, dR, intersectionsFoundR); // skip ray filtering
        }

        else {
            for (int i = 0; i < rays.size(); i++) {
                float dist;
                if (V0.rayIntersection(rays[i], dist) == true) { // if ray intersects bounding volume

                    if (dist < dR[rayIndices[i]]) {
                        R0.push_back(rays[i]);
                        rayIndices2.push_back(rayIndices[i]);
                    }
                }
            }
            DACRT(R0, rayIndices2, V0, T0, scene, trianglesR, uR, vR, dR, intersectionsFoundR);
        }

        if (alpha1 > 0.5) { 
            DACRT(rays, rayIndices, V1, T1,scene, trianglesR, uR, vR, dR, intersectionsFoundR); // skip ray filtering
        }
        else {
            std::vector <Ray> R1;
            std::vector<int> rayIndices22;
            for (int i = 0; i < rays.size(); i++) {
                float dist;
                if (V1.rayIntersection(rays[i], dist)) { // if ray intersects bounding volume
                    if (dist < dR[rayIndices[i]]) {
                        R1.push_back(rays[i]);
                        rayIndices22.push_back(rayIndices[i]);
                    }
                }
            }
            DACRT(R1, rayIndices22, V1, T1, scene, trianglesR, uR, vR, dR, intersectionsFoundR);
        }
    }
};
