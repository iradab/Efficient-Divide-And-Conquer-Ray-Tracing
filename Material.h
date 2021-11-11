#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <exception>

#include "Vec3.h"
const float  PI = 3.14159265358979f;

class Material {
public:
	inline Material() {}

	inline Material(Vec3f& albedo, float diffuse_coeff, float roughness, const Vec3f& refractionIndex) :
		m_albedo(albedo), m_diffuseCoeff(diffuse_coeff), m_roughness(roughness), m_refractionIndex(refractionIndex) {}

	virtual ~Material() {}

	Vec3f evaluateColorResponse(const Vec3f& normal, const Vec3f& wi) const {
		return (material_albedo() * abs(dot(normal, wi))); 
	}

	float specularD (const Vec3f& normalV, Vec3f& wh) const{
		return (m_roughness * m_roughness) / (PI * pow(pow(dot(normalV, wh), 2)*((m_roughness * m_roughness) - 1) +1, 2));
	}

	float specularF(const Vec3f& wi, const Vec3f& wo, Vec3f& wh) const{
		float F0 = 1.f;
		float zero_value = 0;
		return (F0 + (1 - F0)*pow(1-std::max(zero_value,dot(wi, wh)),5));
	}

	float minimum(float& a, float& b, float& c) const {
		return std::min(std::min(a, b), c);
	}

	float specularG(const Vec3f& normalV, const Vec3f& wi, const Vec3f& wo, const Vec3f& wh) const{
		float shadowing = (2*dot(normalV, wh)*dot(normalV, wi)) / dot(wo, wh);
		float masking = (2*dot(normalV, wh)*dot(normalV, wo)) / dot(wo, wh);
		float one = 1.f;
		return minimum(one, shadowing, masking);
	}

	Vec3f halfVector(const Vec3f& wi, const Vec3f& wo) const{
		return (wi + wo) / (wi + wo).length();
	}
	Vec3f evaluateColorResponse(const Vec3f& normalV, const Vec3f& wi, const Vec3f& wo) const {
		// microfacet BRDF model
		Vec3f wh = halfVector(wi, wo);
		float fd = m_diffuseCoeff / PI; // diffuse term 
		float fs = (specularD(normalV, wh) *specularF(wi, wo, wh)* specularG(normalV, wi, wo, wh)) / (4*dot(normalV, wi)*dot(normalV, wo)); // specular term

		return m_albedo * (fd + fs) * abs(dot(normalV, wi));
	}


	Vec3f& material_albedo()		   { return m_albedo; }
	const Vec3f& material_albedo() const   { return m_albedo; }

	float material_diffcoeff()	   { return m_diffuseCoeff; }
	const float material_diffcoeff() const { return m_diffuseCoeff; }

	Vec3f m_albedo;
	Vec3f m_refractionIndex;
	float m_diffuseCoeff;
	float m_roughness; 
};