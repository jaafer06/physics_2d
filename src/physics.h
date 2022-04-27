#pragma once
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <vector>
#include <set>
constexpr float pi = 3.14159265358979323846;
static const Eigen::Rotation2Df R_90{ pi / 2 };

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
	
	void update(float deltaTime) {
		translate(linearVelocity * deltaTime);
		rotate(angularVelocity * deltaTime);
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

		if (points.size() == 3) {
			const std::array<Eigen::Vector2f, 3> edges{ points[0] - points[1], points[0] - points[2] , points[1] - points[2] };
			const std::array<Eigen::Vector2f, 3> edgeIndexToOuterPoint{ points[2], points[1], points[0] };
			const std::array<float, 3> distances{ edges[0].squaredNorm(), edges[1].squaredNorm(), edges[2].squaredNorm() };
			std::array<unsigned, 3> indices{ 0, 1, 2 };
			std::sort(indices.begin(), indices.end(), [&distances](const auto i1, const auto i2) {
				return distances[i1] < distances[i2];
			});
			const Eigen::Vector2f normal = R_90 * edges[indices[2]].normalized();
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
			return { points, normal, depth, IntersectionType::EdgeEdge, true };

		} else if (points.size() == 2) {
			const auto direction = points[0] - points[1];
			const Eigen::Vector2f normal = R_90 * direction.normalized();
			const auto depth = 0;
			return { points, normal, depth, IntersectionType::EdgeEdge, true };
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

	void translate(Eigen::Vector2f t) {
		position = position + t;
	};

	void setPosition(const Eigen::Vector2f& _pos) {
		position = _pos;
	}
	const Eigen::Vector2f& getPosition() const {
		return position;
	}

	void setLinearVelocity(const Eigen::Vector2f v) {
		linearVelocity = v;
	}

	void setAngularVelocity(const float w) {
		angularVelocity = w;
	}

	void rotate(const float deltaAngle) {
		angle += deltaAngle;
	}
	void addLinearVelocity(const Eigen::Vector2f v) {
		linearVelocity = linearVelocity + v;
	}

	Eigen::Vector2f getParticuleVelocity(const Eigen::Vector2f& p) {
		if (!contains(p)) {
			std::cout << "hm" << std::endl;
		};
		Eigen::Matrix2f A{ {0, - angularVelocity}, {angularVelocity, 0} };
		const auto vAngle = A * (p - position);
		return vAngle + linearVelocity;
	}
	float mass{ 1 };
private:
	Eigen::Vector2f position;
	Eigen::Vector2f size;
	Eigen::Vector2f linearVelocity{ 0, 0 };
	float angle{ 0 };
	float angularVelocity{ 0 };
};


class CollisionResolver {
	using Collision = std::vector<std::tuple<Rect::IntersectionResult, unsigned, unsigned>>;
public:
	void resolveInterPenetration(Rect& r1, Rect& r2, const Rect::IntersectionResult& result) {
		if (result.intersects && result.depth > 0) {
			const auto [n1, _] = getNormal(r1, r2, result);
			const auto totalMass = r1.mass + r2.mass;
			r1.translate(n1 * result.depth * (r2.mass / totalMass));
			r2.translate(-n1 * result.depth * (r1.mass / totalMass));
		}
	}

	void resolveMovement(Rect& r1, Rect& r2, const Rect::IntersectionResult& result) {
		if (!result.intersects) {
			return;
		}
		const auto [n, _] = getNormal(r1, r2, result);
		const Eigen::Vector2f t = R_90 * n;
		const auto& contactPoint = result.contactPoints[0];
		const auto v1 = r1.getParticuleVelocity(contactPoint);
		const auto v2 = r2.getParticuleVelocity(contactPoint);
		const float v1_n = v1.dot(n);
		const float v1_t = v1.dot(t);
		const float v2_n = v2.dot(n);
		const float v2_t = v2.dot(t);
		//const auto totalVelocity = r1.getParticuleVelocity(contactPoint) + 
		const Eigen::Matrix2f A{ {r1.mass, r2.mass}, {1, -1} };
		const Eigen::Vector2f b{ r1.mass * v1_n + r2.mass * v2_n, v2_n - v1_n};
		const Eigen::Vector2f sol = A.colPivHouseholderQr().solve(b);

		const Eigen::Vector2f v1f = n * sol[0] + v1_t * t;
		const Eigen::Vector2f v2f = n * sol[1] + v2_t * t;
		setLinearAndAngularVelocity(r1, result.contactPoints[0], v1f);
		setLinearAndAngularVelocity(r2, result.contactPoints[0], v2f);
	};
	

private:

	void setLinearAndAngularVelocity(Rect& r, const Eigen::Vector2f contactPoint, const Eigen::Vector2f velocity) {
		const auto d = (contactPoint - r.getPosition());
		const auto linearVelocity = (velocity.dot(d) / d.dot(d)) * d;
		const auto rotationalVelocityComponent = velocity - linearVelocity;
		const auto w = (R_90 * d).normalized().dot(rotationalVelocityComponent) /100;
		r.setAngularVelocity(w);
		r.setLinearVelocity(linearVelocity);
		std::cout << w << "  " << linearVelocity[0] << " " << linearVelocity[1] << std::endl;
	}

	std::tuple<Eigen::Vector2f, Eigen::Vector2f> getNormal(const Rect& rect1, const Rect& rect2, const Rect::IntersectionResult& result) {
		const auto normal = result.normal.normalized();
		if ((rect1.getPosition() - result.contactPoints[0]).dot(normal) > 0) {
			return { normal, -normal };
		};
		return { -normal, normal };
	}

};

void update(std::vector<Rect*> rects, const float dt) {
	for (const auto r : rects) {
		r->update(dt);
	}
};