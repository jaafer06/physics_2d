#pragma once
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <vector>
#include <set>
constexpr float pi = 3.14159265358979323846;

struct Edge {
	Eigen::Vector2f start;
	Eigen::Vector2f direction;
	struct IntersectionResult {
		float depth;
		Eigen::Vector2f intersectionPoint;
		bool intersects;
	};

	Edge::IntersectionResult intersects(const Edge& other) const {
		const Eigen::Vector2f b = other.start - start;
		Eigen::Matrix2f A;
		A.col(0) = direction;
		A.col(1) = -other.direction;
		const auto result = A.colPivHouseholderQr().solve(b);

		const bool intersects = result[0] < 1 && result[0] > 0 && result[1] < 1 && result[1] > 0;
		const float depth = result[1] * other.direction.norm();
		return { depth, start + result[0] * direction, intersects };
	}

	Eigen::Vector2f project(const Eigen::Vector2f p) {
		const auto v = p - start;
		return (v.dot(direction) / direction.dot(direction)) * direction + start;
	}
	//bool isOverlayed(const Edge& other) const {
	//	return (get_a_b_LineCoeff() - other.get_a_b_LineCoeff()).cwiseAbs().maxCoeff() < 0.01;
	//}

	//inline Eigen::Vector2f get_a_b_LineCoeff() const {
	//	const float a = (p2[1] - p1[1]) / (p2[0] - p1[0]);
	//	const float b = p1[1] - a * p1[0];
	//	return { a, b };
	//}
};


class Rect {
public:
	enum class IntersectionType {
		PointEdge, EdgeEdge, PointPoint
	};

	struct IntersectionResult {
		std::vector<Eigen::Vector2f> contactPoints;
		Eigen::Vector2f normal;
		float depth;
		IntersectionType type;
		bool intersects;
	};

	Rect(float x, float y, float size_x, float size_y)
		: position{ x, y }, size{ size_x, size_y }, angle{ 0 } {
	}

	IntersectionResult intersects(const Rect& other) const {
		auto const otherPoints = other.getPoints();
		auto const points = getPoints();
		std::vector<Eigen::Vector2f> contactPoints;
		for (unsigned otherPointIndex = 0; otherPointIndex < 4;  ++otherPointIndex) {
			const auto otherPoint = otherPoints[otherPointIndex];
			if (contains(otherPoint)) {
				contactPoints.push_back(otherPoint);
			}
		}

		for (unsigned pointIndex = 0; pointIndex < 4; ++pointIndex) {
			const auto point = points[pointIndex];
			if (other.contains(point)) {
				contactPoints.push_back(point);
			}
		}

		for (const auto& edge : getEdges()) {
			for (const auto& otherEdge: other.getEdges()) {
				auto const r = edge.intersects(otherEdge);
				if (r.intersects) {
					contactPoints.push_back(r.intersectionPoint);
				}
			}
		}

		return processIntersection(contactPoints);

	};

	IntersectionResult processIntersection(const std::vector<Eigen::Vector2f>& points) const {
		const Eigen::Rotation2Df R(pi / 2);
		if (points.size() == 3) {
			const std::array<Eigen::Vector2f, 3> edges{ points[0] - points[1], points[0] - points[2] , points[1] - points[2] };
			const std::array<Eigen::Vector2f, 3> edgeIndexToOuterPoint{ points[2], points[1], points[0] };
			const std::array<float, 3> distances{ edges[0].squaredNorm(), edges[1].squaredNorm(), edges[2].squaredNorm() };
			std::array<unsigned, 3> indices{ 0, 1, 2 };
			std::sort(indices.begin(), indices.end(), [&distances](const auto i1, const auto i2) {
				return distances[i1] < distances[i2];
			});
			const Eigen::Vector2f normal = R * edges[indices[2]].normalized();
			const float depth = edges[indices[0]].norm();
			const std::vector<Eigen::Vector2f> contactPoints{ edgeIndexToOuterPoint[indices[2]] };
			return { contactPoints, normal, depth, IntersectionType::PointEdge, true };
		} else if (points.size() == 4) {
			std::array<unsigned, 3> indices{ 0, 1, 2 };
			const auto d0 = points[0] - points[1];
			const auto d1 = points[0] - points[2];
			const auto d2 = points[0] - points[3];
			const std::array<Eigen::Vector2f, 3> v{ d0, d1, d2 };
			const std::array<float, 3> distances{ d0.squaredNorm(), d1.squaredNorm(), d2.squaredNorm() };
			std::sort(indices.begin(), indices.end(), [&distances](const auto i1, const auto i2) {
				return distances[i1] < distances[i2];
			});
			const auto normal = v[indices[0]].normalized();
			const float depth = v[indices[0]].norm();
			const std::vector<Eigen::Vector2f> contactPoints = points;
			return { contactPoints, normal, depth, IntersectionType::EdgeEdge, true };

		} else if (points.size() == 2) {
			const auto direction = points[0] - points[1];
			const Eigen::Vector2f normal = R * direction.normalized();
			const auto depth = 0;
			const std::vector<Eigen::Vector2f> contactPoints = points;
			return { contactPoints, normal, depth, IntersectionType::EdgeEdge, true };
		} else if (points.size() == 0) {
			return { {}, {}, 0, IntersectionType::EdgeEdge, false };
		}
		throw "hm";
	}

	bool contains(const Eigen::Vector2f& point) const {
		auto const pts = getPoints();
		Eigen::Matrix2f A;
		A.col(0) = pts[1] - pts[0];
		A.col(1) = pts[3] - pts[0];
		const auto b = point - pts[0];
		const auto result = A.colPivHouseholderQr().solve(b);
		return result.maxCoeff() <= 1 && result.minCoeff() >= 0;
	};


	std::tuple<unsigned,Edge::IntersectionResult> edgeIntersection(const Edge& otherEdge) const {
		const auto edges = getEdges();
		for (unsigned edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
			const auto edge = edges[edgeIndex];
			const auto result = edge.intersects(otherEdge);
			if (result.intersects) {
				return std::tuple{ edgeIndex, result };
			}
		};
		return std::tuple{ 0, Edge::IntersectionResult{ 0, {}, false } };
	};

	std::array<Eigen::Vector2f, 4> getPoints() const {
		const auto R = Eigen::Rotation2Df(angle);
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
		const Edge e0{ pts[0], pts[1] - pts[0] };
		const Edge e1{ pts[1], pts[2] - pts[1] };
		const Edge e2{ pts[2], pts[3] - pts[2] };
		const Edge e3{ pts[3], pts[0] - pts[3] };
		return { e0, e1, e2, e3 };
	};

	void rotate(float _angle) {
		angle = _angle;
	}

	void addRotation(float deltaAngle) {
		angle += deltaAngle;
	}

	std::array<Edge, 2> vertexIndexToEdge(const unsigned index) const {
		auto const pts = getPoints();
		if (index == 0) {
			const Edge v1{ pts[index],pts[1] - pts[index] };
			const Edge v2{ pts[index], pts[3] - pts[index] };
			return { v1, v2 };
		}
		else if (index == 1) {
			const Edge v1{ pts[index], pts[0] - pts[index] };
			const Edge v2{ pts[index], pts[2] - pts[index] };
			return { v1, v2 };
		}
		else if (index == 2) {
			const Edge v1{ pts[index], pts[3] - pts[index] };
			const Edge v2{ pts[index], pts[1] - pts[index] };
			return { v1, v2 };
		}
		else if (index == 3) {
			const Edge v1{ pts[index], pts[2] - pts[index] };
			const Edge v2{ pts[index], pts[0] - pts[index] };
			return { v1, v2 };
		}
		throw "index > 3";
	};

	Eigen::Vector2f getCenterOfMass() const {
		const auto points = getPoints();
		return  Eigen::Map<const Eigen::Matrix<float, 4, 2, Eigen::RowMajor>>((float*)points.data()).colwise().sum() * 0.25;
	};

	void translate(Eigen::Vector2f t) {
		position = position + t;
	};

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
	using Collision = std::vector<std::tuple<Rect::IntersectionResult, unsigned, unsigned>>;
public:
	void resolveInterPenetration(Rect& r1, Rect& r2, const Rect::IntersectionResult& result) {
		if (result.intersects && result.depth > 0) {
			const auto [n1, n2] = getNormal(r1, r2, result);
			r1.translate(n1 * result.depth);
		}
	}
private:

	std::tuple<Eigen::Vector2f, Eigen::Vector2f> getNormal(const Rect& rect1, const Rect& rect2, const Rect::IntersectionResult& result) {
		const auto normal = result.normal;
		if ((rect1.getCenterOfMass() - result.contactPoints[0]).dot(normal) > 0) {
			return { normal, -normal };
		};
		return { -normal, normal };
	}

};