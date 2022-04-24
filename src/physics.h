#pragma once
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <vector>
#include <set>

struct Edge {
	Eigen::Vector2f p1;
	Eigen::Vector2f p2;
	struct IntersectionResult {
		float k1;
		float k2;
		bool intersects;
	};

	void flip() {
		std::swap(p1, p2);
	}

	Edge::IntersectionResult intersects(const Edge& other) const {
		const Eigen::Vector2f v1 = p2 - p1;
		const Eigen::Vector2f v2 = -(other.p2 - other.p1);
		const Eigen::Vector2f b = other.p1 - p1;
		Eigen::Matrix2f A;
		A.col(0) = v1;
		A.col(1) = v2;
		const auto result = A.colPivHouseholderQr().solve(b);
		const bool intersects = result[0] < 1 && result[0] > 0 && result[1] < 1 && result[1] > 0;
		const float k1 = result[0] * v1.norm();
		const float k2 = result[1] * v2.norm();
		return { k1 , k2, intersects };
	}

	bool isOverlayed(const Edge& other) const {
		return (get_a_b_LineCoeff() - other.get_a_b_LineCoeff()).cwiseAbs().maxCoeff() < 0.01;
	}

	inline Eigen::Vector2f get_a_b_LineCoeff() const {
		const float a = (p2[1] - p1[1]) / (p2[0] - p1[0]);
		const float b = p1[1] - a * p1[0];
		return { a, b };
	}
};


class Rect {
public:
	enum class IntersectionType {
		PointEdge, EdgeEdge, PointPoint
	};

	struct IntersectionResult {
		std::vector<Eigen::Vector2f> intersectionPoints;
		Eigen::Vector2f normal;
		float depth;
		bool intersects;
		IntersectionType type;
	};

	Rect(float x, float y, float size_x, float size_y)
		: position{ x, y }, size{ size_x, size_y }, angle{ 0 } {
	}

	const IntersectionResult intersects(const Rect& other) const {
		auto const otherPoints = other.getPoints();
		auto const pts = getPoints();
		bool thisPrimary = contains(otherPoints[0]) || contains(otherPoints[1]) || contains(otherPoints[2]) || contains(otherPoints[3]);
		if (thisPrimary) {
			hm(other);
		}
		float otherPrimary = other.contains(pts[0]) || other.contains(pts[1]) || other.contains(pts[2]) || other.contains(pts[3]);
		if (otherPrimary) {
			other.hm(*this);
		};
		return { {}, Eigen::Vector2f{ 0, 0 }, 0, false, IntersectionType::PointPoint };
	};

	bool contains(const Eigen::Vector2f& point) const {
		auto const pts = getPoints();
		Eigen::Matrix2f A;
		A.col(0) = pts[1] - pts[0];
		A.col(1) = pts[3] - pts[0];
		const auto b = point - pts[0];
		const auto result = A.colPivHouseholderQr().solve(b);
		return result.maxCoeff() <= 1 && result.minCoeff() >= 0;
	};

	IntersectionResult hm(const Rect& other) const {
		std::array<std::array<float, 2>, 4> distances{ 0 };
		std::set<unsigned> activeEdges;
			
		auto otherEdges = other.getEdges();
		const auto edges = getEdges();
		for (unsigned edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
			for (unsigned otherEdgeIndex = 0; otherEdgeIndex < 4; ++otherEdgeIndex) {
				const auto edge = edges[edgeIndex];
				auto otherEdge = otherEdges[otherEdgeIndex];
				if (!contains(otherEdge.p1)) {
					otherEdge.flip();
				}
				const auto result = otherEdge.intersects(edge);
				if (result.intersects) {
					std::cout << edgeIndex << std::endl;
					activeEdges.insert(edgeIndex);
					if (distances[edgeIndex][0] != 0) {
						distances[edgeIndex][1] = result.k1;
					} else {
						distances[edgeIndex][0] = result.k1;
					}
				};

			}
		};
		for (const auto arr : distances) {
			std::cout << "{ ";
			for (auto d : arr) {
				std::cout << d << " ";
			};
			std::cout << "} ";
		};
		std::cout << std::endl;

		//for (const auto a : activeEdges) {
		//	std::cout << a << " ";
		//};
		//std::cout << std::endl;

		if (activeEdges.size() == 1) {
			std::cout << *(activeEdges.begin()) << std::endl;
			const auto normal = edgeIndexToNormal[*(activeEdges.begin())];
			return {};
		} else if (activeEdges.size() == 2) {
			const unsigned edge1 = *(activeEdges.begin());
			const unsigned edge2 = *(++activeEdges.begin());
			if (distances[edge1][0] < distances[edge2][0]) {
				const auto normal = edgeIndexToNormal[edge1];
			} else if (distances[edge1][0] > distances[edge2][0]) {
				const auto normal = edgeIndexToNormal[edge2];
			} else {
				const auto normal = (edgeIndexToNormal[edge2] + edgeIndexToNormal[edge1]).normalized();
			}
		}

	};

	 std::array<Eigen::Vector2f, 4> getPoints() const {
		const auto R = Eigen::Rotation2Df(0.);
		const auto sx = size[0] / 2;
		const auto sy = size[1] / 2;
		const auto v0 = (R * Eigen::Vector2f{-sx , sy}) + position;
		const auto v1 = (R * Eigen::Vector2f{ sx , sy }) + position;
		const auto v2 = (R * Eigen::Vector2f{ sx , -sy }) + position;
		const auto v3 = (R * Eigen::Vector2f{ -sx , -sy }) + position;
		return { v0, v1, v2, v3 };
	};

	std::array<Edge, 4> getEdges() const {
		const auto pts = getPoints();
		const Edge e0{ pts[0], pts[1] };
		const Edge e1{ pts[1], pts[2] };
		const Edge e2{ pts[2], pts[3] };
		const Edge e3{ pts[3], pts[0] };
		return { e0, e1, e2, e3 };
	};

	void rotate(float _angle) {
		angle = _angle;
	}

private:
	inline static Eigen::Vector2f vertexIndexToNormal[4][4] {
		{Eigen::Vector2f{0, 0}, Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{-1, 0}},
		{Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{1, 0}, Eigen::Vector2f{0, 0}},
		{Eigen::Vector2f{0, 0}, Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, -1}, Eigen::Vector2f{0, 0}},
		{Eigen::Vector2f{-1, 0}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{-1, 0}, Eigen::Vector2f{0, 0}},
	};

	inline static Eigen::Vector2f edgeIndexToNormal[4] {
		Eigen::Vector2f{0, 1}, Eigen::Vector2f{1, 0}, Eigen::Vector2f{0, -1}, Eigen::Vector2f{-1, 0}
	};

	Eigen::Vector2f position;
	Eigen::Vector2f size;
	float angle;

};



class CollisionResolver {
public:

private:

};