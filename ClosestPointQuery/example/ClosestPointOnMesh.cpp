#define TINYOBJLOADER_IMPLEMENTATION
#define ENABLE_MULTITHREADING
#define ASYNC_TASK_COUNT 256
#define QUERY_POINT_COUNT 100000
#define VISUALIZER_QUERY_POINTS
#define VISUALIZER_BOUNDING_BOXES
//#ifdef DEBUG
#define PRINT_TIME(msg, t) std::cout << msg << " (Time: " << t << "ms)\n";
//#else
//#define PRINT_TIME(msg, t)
//#endif

#include <iostream>
#include <fstream>
#include <future>
#include <random>
#include <thread>
#include <tiny_obj_loader.h>
#include <ClosestPointQuery.h>

using namespace geoutils;

// Constants declarations
const char* MODEL_PATH = "../../../Assets/head.obj";
const char* VISUALIZER_CSV_PATH = "../../../Visualizer/query_points.csv";

// Forward declarations
double random_double(double min, double max);
Vec3 random_in_unit_sphere();
std::vector<Mesh> load_obj_model(const char* model_path);

// A scoped timer class for profiling execution time.
// Timer starts once created and stops when it runs out of scope.
// Example:
//	Timer timer;
//	complex_function_call();
//	std::cout << "Time elapsed: " << timer.elapsed_ms() << "ms";
class Timer {
private:
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point last_requested_time;
public:
	Timer() : start{ std::chrono::high_resolution_clock::now() }, last_requested_time{ start } {}
	double elapsed_ms() const { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0; }
	double delta_ms() {
		const auto now = std::chrono::high_resolution_clock::now();
		const double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(now - last_requested_time).count() / 1000.0;
		last_requested_time = now;
		return delta_time;
	}
};

int main(void) {
	std::vector<Mesh> meshes;
	{
		Timer load_obj_timer;
		// Load model from file
		try {
			meshes = load_obj_model(MODEL_PATH);
		}
		catch (std::exception e) {
			std::cerr << e.what();
			return -1;
		}
		PRINT_TIME("Loading model", load_obj_timer.elapsed_ms());
	}

	// Generate random query points around the model
	std::vector<std::pair<float, Point>> query_points(QUERY_POINT_COUNT);
	for (size_t i = 0; i < QUERY_POINT_COUNT; ++i) query_points[i] = { 0.5f, random_in_unit_sphere() * 1.5f };

	// Start the query!
	std::vector<std::pair<bool, Point>> closest_points(query_points.size());
	{
		Timer elapsed_timer;
		for (const Mesh& mesh : meshes) {
			ClosestPointQuery query(mesh);
			PRINT_TIME("Construct ClosestPointQuery", elapsed_timer.delta_ms());
			size_t async_task_count = ASYNC_TASK_COUNT;
			for (size_t i = 0; i < query_points.size(); i += ASYNC_TASK_COUNT) {
#ifdef ENABLE_MULTITHREADING
				async_task_count = std::min((size_t)ASYNC_TASK_COUNT, query_points.size() - i);
				std::vector<std::future<bool>> threads(async_task_count);
				for (size_t j = 0; j < async_task_count; ++j) {
					threads[j] = std::async(std::launch::async,
						&ClosestPointQuery::operator(), &query,
						query_points[i + j].second,				// Query point position
						query_points[i + j].first, 				// Maximum query distance
						std::ref(closest_points[i + j].second)  // Output closest point
					);
				}
				for (size_t j = 0; j < async_task_count; ++j) {
					closest_points[i + j].first = threads[j].get();
				}
#else
				closest_points[i].first = query(
					query_points[i].second,		// Query point position
					query_points[i].first, 		// Maximum query distance
					closest_points[i].second	// Output closest point
				);
#endif
			}
			PRINT_TIME("Querying " + std::to_string(query_points.size()) + " points on " + std::to_string(mesh.indices.size() / 3) + " triangles", elapsed_timer.delta_ms());
		}
	}

#ifdef VISUALIZER_QUERY_POINTS
	#define VISUALIZER_BOUNDING_BOXES
	// Output the results to a CSV file.
	std::ofstream query_points_csv(VISUALIZER_CSV_PATH, std::ofstream::trunc);
	if (query_points_csv.is_open()) {
		query_points_csv << MODEL_PATH << "\n";
		for (size_t i = 0; i < query_points.size(); ++i) {
			query_points_csv << query_points[i].first << "," << query_points[i].second.x() << "," << query_points[i].second.y() << "," << query_points[i].second.z() << ",";
			query_points_csv << closest_points[i].first << "," << closest_points[i].second.x() << "," << closest_points[i].second.y() << "," << closest_points[i].second.z() << "\n";
		}
		query_points_csv.close();
	}
#endif
}

// Utility function for loading OBJ model from file and transform it into an array of geoutils::Mesh.
// The path is relative to the project root directory when using IDE, but relative to executable when packaged.
std::vector<Mesh> load_obj_model(const char* model_path) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn, err;
	if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, model_path)) {
		throw std::runtime_error(warn + err);
	}
	std::vector<Mesh> meshes(shapes.size());
	for (size_t i = 0; i < shapes.size(); ++i) {
		meshes[i].vertices.reserve(attrib.vertices.size() / 3);
		for (size_t j = 0; j < attrib.vertices.size(); j += 3) {
			meshes[i].vertices.push_back(Point(attrib.vertices[j], attrib.vertices[j + 1], attrib.vertices[j + 2]));
		}
		meshes[i].indices.reserve(shapes[i].mesh.indices.size());
		for (const auto& index : shapes[i].mesh.indices) {
			meshes[i].indices.push_back(index.vertex_index);
		}
	}
	return meshes;
}

inline double random_double(double min, double max) {
	static std::uniform_real_distribution<double> distribution(0.0, 1.0);
	static std::mt19937 generator;
	return min + (max - min) * distribution(generator);
}
// Naive way to generate random samples inside a unit sphere.
// [Source code]. https://raytracing.github.io/books/RayTracingInOneWeekend.html
Vec3 random_in_unit_sphere() {
	while (true) {
		Vec3 p = Vec3(random_double(-1, 1), random_double(-1, 1), random_double(-1, 1));
		if (p.length2() >= 1.f) continue;
		return p;
	}
}