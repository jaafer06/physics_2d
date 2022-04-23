#pragma once
#include <Eigen/Dense>
#include <array>
#include <algorithm>


struct IntersectionResult {
	bool intersects;
	Eigen::Vector2f intersectionPoint;
	Eigen::Vector2f normal;
};

class Rect {
public:
	Rect(float x, float y, float size_x, float size_y)
		: position{ x, y }, size{ size_x, size_y } {

	}
	IntersectionResult intersectionTest(const Rect& other) const {
		IntersectionResult result;
		oneWayIntersects(other, result);
		if (result.intersects) {
			return result;
		};
		other.oneWayIntersects(*this, result);
		return result;
	}

	bool contains(const Eigen::Vector2f& point) const {
		return position[0] < point[0] && position[0] + size[0] > point[0] &&
			position[1] < point[1] && position[1] + size[1] > point[1];
	}

	bool contains(float x, float y) const {
		return contains(Eigen::Vector2f{ x, y });
	}

private:
	inline static Eigen::Vector2f vertexIndexToNormal[4][4] {
		{Eigen::Vector2f{0, 0}, Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{-1, 0}},
		{Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{1, 0}, Eigen::Vector2f{0, 0}},
		{Eigen::Vector2f{0, 0}, Eigen::Vector2f{0, 1}, Eigen::Vector2f{0, -1}, Eigen::Vector2f{0, 0}},
		{Eigen::Vector2f{-1, 0}, Eigen::Vector2f{0, 0}, Eigen::Vector2f{-1, 0}, Eigen::Vector2f{0, 0}},
	};
	Eigen::Vector2f centerOfMass;
	Eigen::Vector2f position;
	Eigen::Vector2f size;

	std::tuple<bool, Eigen::Vector2f> hasVertexInside(const Rect& other) const {
		if (contains(other.position)) {
			return { true, other.position };
		};
		if (contains(other.position + other.size)) {
			return { true, other.position + other.size };
		};
		const Eigen::Vector2f uppderRight = other.position + Eigen::Vector2f(other.size[0], 0);
		if (contains(uppderRight)) {
			return { true, uppderRight };
		};
		const Eigen::Vector2f lowerRight = other.position + Eigen::Vector2f(0, other.size[1]);
		if (contains(lowerRight)) {
			return { true, lowerRight };
		};
		return { false, { 0, 0 } };
	};

	void oneWayIntersects(const Rect& other, IntersectionResult& result) const {
		const auto [intersects, p] = hasVertexInside(other);
		if (intersects) {
			const Eigen::Vector4f distance = (other.points().rowwise() - p.transpose()).rowwise().squaredNorm().transpose();
			std::array<unsigned, 4> indices = { 0, 1, 2, 3 };
			std::sort(indices.begin(), indices.end(), [&distance](const auto i, const auto j) {return distance[i] > distance[j]; });
			result = { intersects, p, vertexIndexToNormal[indices[0]][indices[1]] };
			return;
		};
		result = { false, {0, 0}, {0, 0} };
	}

	inline Eigen::Matrix<float, 4, 2> points() const {
		const auto upperRight = position + Eigen::Vector2f(size[0], 0);
		const auto diagonal = position + size;
		const auto lowerRight = position + Eigen::Vector2f(0, size[1]);
		Eigen::Matrix<float, 4, 2> p;
		p << position, upperRight, diagonal, lowerRight;
		//p.block<1,2>(0, 0) = position;
		//p.block<1,2>(1, 0) = upperRight;
		//p.block<1,2>(2, 0) = diagonal;
		//p.block<1,2>(3, 0) = lowerRight;
		return p;
	};
};



class CollisionResolver {
public:

private:

};