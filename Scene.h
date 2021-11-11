#pragma once

#include <vector>

#include "Camera.h"
#include "Mesh.h"
#include "LightSource.h" 

class Scene {
public:
	inline Scene() {}
	virtual ~Scene() {}

	inline const Camera& camera() const { return m_camera; }

	inline Camera& camera() { return m_camera; }

	inline const std::vector<Mesh>& meshes() const { return m_meshes; }

	inline std::vector<Mesh>& meshes() { return m_meshes; }

	inline const std::vector<LightSource>& light_sources() const { return m_lightSources; }

	inline std::vector<LightSource>& light_sources() { return m_lightSources; }

private:
	Camera m_camera;
	std::vector<Mesh> m_meshes;
	std::vector<LightSource> m_lightSources;
};