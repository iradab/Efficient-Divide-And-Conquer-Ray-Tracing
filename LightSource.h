#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <exception>

#include "Vec3.h"

class LightSource {
public:
	inline LightSource(const Vec3f& position, const Vec3f& color, float intensity) :
		m_position(position), m_color(color), m_intensity(intensity) {}

	virtual ~LightSource() {}

	Vec3f& source_position() { return m_position; }
	const Vec3f& source_position() const { return m_position; }

	Vec3f& source_color() { return m_color; }
	const Vec3f& source_color() const { return m_color; }


	float& source_intensity() { return m_intensity; }
	const float& source_intensity() const { return m_intensity; }


protected:
	Vec3f m_position;
	Vec3f m_color;
	float m_intensity;
};