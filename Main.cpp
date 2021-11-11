#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <exception>

#include "CommandLine.h"
#include "Image.h"
#include "Ray.h"
#include "Camera.h"
#include "Mesh.h"
#include "Scene.h"
#include "RayTracer.h"
#include "Material.h"
#include "LightSource.h"
using namespace std;

int main(int argc, char** argv) {

	CommandLine args;
	if (argc > 1) {
		try {
			args.parse(argc, argv);
		}
		catch (const std::exception& e) {
			std::cerr << e.what() << std::endl;
			args.printUsage(argv[0]);
			exit(1);
		}
	}
	// Initialization

	Image image(args.width(), args.height());
	std::cout << "Width = " << args.width() << " , height = " << args.height() << std::endl;
	Scene scene;

	Camera camera(Vec3f(0.f, 0.f, 1.5f),
		Vec3f(),
		Vec3f(0.f, 1.f, 0.f),
		60.f,
		float(args.width()) / args.height());


	scene.camera() = camera;

	// Loading a mesh
	Mesh mesh;
	LightSource light_source = LightSource(Vec3f(1.f, 1.f, 1.f), Vec3f(1.f, 1.f, 1.f), 1.f);
	scene.light_sources().push_back(light_source);

	try {
		mesh.loadOFF("../example.off");
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		exit(1);
	}

	Material baseMaterial = Material(Vec3f(1.f, 1.f, 1.f), 1.f, 1.f, Vec3f(1.f, 1.f, 1.f));
	mesh.setMaterial(baseMaterial);

	scene.meshes().push_back(mesh);

	RayTracer rayTracer;
	 
	// Rendering

	image.fillBackground();
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	rayTracer.render(scene, image); 
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	image.savePPM(args.outputFilename());
	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;

	return 0;
}
