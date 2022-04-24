#pragma once
#include <vector>
#include <Eigen/dense>
#include <string>
#include <fstream>
#include <string>
#include <sstream>
#include "utils/objloader.hpp"

namespace utils {

	class Mesh {
	public:
		struct Triangle {
			unsigned int index0;
			unsigned int index1;
			unsigned int index2;
		};

		struct Vertex {
			Eigen::Vector4f position;
			Eigen::Vector3f normal;
			Eigen::Vector4f color;
		};

		Mesh() {}

		Mesh(std::vector<Vertex> vertices, std::vector<Triangle> triangles)
			: vertices(vertices), triangles(triangles) {}

		bool loadObj(const std::string& file_name) {
			std::vector<float> result;
			objl::Loader loader;
			if (!loader.LoadFile(file_name)) {
				return false;
			};
			for (auto& vertex : loader.LoadedVertices) {
				Vertex v;
				v.color = { 1., 0., 0., 1. };
				v.position = { vertex.Position.X, vertex.Position.Y, vertex.Position.Z, 1 };
				v.normal = { vertex.Normal.X, vertex.Normal.Y, vertex.Normal.Z };
				vertices.push_back(v);
			};
			for (int index = 0; index < loader.LoadedIndices.size(); index += 3) {
				Triangle t{ loader.LoadedIndices[index], loader.LoadedIndices[index + 1], loader.LoadedIndices[index + 2] };
				triangles.push_back(t);
			}
			return true;
		}

		void preprocessNormalize() {
			auto [center, radius] = getMeshCenterAndRadius();
			for (auto& vertex : getVertices()) {
				vertex.position.head(3) = (vertex.position.head(3) - center) / radius;
			}
		}

		void preprocessSetColor(const Eigen::Vector4f&& color) {
			for (auto& vertex : getVertices()) {
				vertex.color = color;
			}
		}

		Eigen::Vector3f getMeshCenter() {
			Eigen::Vector3f center{ 0, 0, 0 };
			for (Vertex& point : getVertices()) {
				center += point.position.head(3);
			}
			return center / getVertices().size();
		}

		void preprocessTranslate(const Eigen::Vector3f&& t) {
			for (Vertex& point : getVertices()) {
				point.position.head(3) += t;
			}
		}

		void preprocessTransform(const Eigen::Matrix4f&& matrix) {
			for (Vertex& point : getVertices()) {
				point.position = matrix * point.position;
			}
		}

		void preprocessScale(float xScale, float yScale, float zScale) {
			auto center = getMeshCenter().head(3);
			Eigen::Matrix3f M = Eigen::Matrix3f::Identity();
			M(0, 0) = xScale;
			M(1, 1) = yScale;
			M(2, 2) = zScale;
			for (Vertex& point : getVertices()) {
				point.position.head(3) = M * point.position.head(3) - M * center + center;
			}
		}
		void preprocessScale(float scale) {
			preprocessScale(scale, scale, scale);
		}

		std::pair<Eigen::Vector3f, float> getMeshCenterAndRadius() {
			Eigen::Vector3f mesh_center = getMeshCenter();
			float mesh_radius = 0;
			for (Vertex& vertex : getVertices()) {
				mesh_radius = std::max((vertex.position.head(3) - mesh_center).norm(), mesh_radius);
			}
			return { mesh_center, mesh_radius };
		}

		std::vector<Triangle>& getTriangles() {
			return triangles;
		}

		std::vector<Vertex>& getVertices() {
			return vertices;
		}

	private:
		std::vector<Vertex> vertices;
		std::vector<Triangle> triangles;
	};
}