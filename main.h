#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_map>
#include "../../MyLibrary/DefaultFunction.h"
#include "../../MyLibrary/NormalMatrix.h"

using namespace Darkness;

using holder = std::unique_ptr<Expression>;

enum class Mode {
	unassigned, system, structure
};


using ang = size_t;//angle 角度
using co = constant;//coordination 坐标
using key = size_t;//键值


enum class joint {
	unknown = -1, lever, pin, rigid
};

class ref_ostream {
public:
	std::ostream &err;
	ref_ostream(std::ostream &e) noexcept:err(e){ }
	~ref_ostream() noexcept = default;

private:

};


class tableName :public ref_ostream{
public:
	tableName(std::ostream &err)noexcept :ref_ostream(err) { }
	~tableName() noexcept = default;
	auto cbegin()const noexcept { return bodies.cbegin(); }
	auto cend()const noexcept { return bodies.cend(); }
	size_t find_name(const char *name)noexcept {
		for (auto iter = bodies.cbegin(); iter != bodies.cend(); ++iter) {
			if (*iter == name) {
				return iter - bodies.cbegin();
			}
		}
		return std::numeric_limits<size_t>::max();
	}
	size_t new_name(const char* name)noexcept {
		for (auto iter = bodies.cbegin(); iter != bodies.cend(); ++iter) {
			if (*iter == name) {
				return std::numeric_limits<size_t>::max();
			}
		}
		try {
			bodies.emplace_back(name);
			return bodies.size() - 1;
		}
		catch (const std::exception &) {
			err << "Failed in emplacing." << std::endl;
			return std::numeric_limits<size_t>::max();
		}
	}
	auto size()const noexcept { return bodies.size(); }
	const auto &operator[](size_t i)const noexcept {
		return this->bodies[i];
	}
private:
	//刚体表 rigid bodies map
	//基础（base）是具有不同性质的刚体 basement is a rigid body, though having different behaviours
	std::vector<std::string> bodies;
};

template<typename external>
class tableExternal {
public:
	tableExternal() = default;
	~tableExternal()noexcept = default;
	template<typename... T>void new_external(key k, T &&... t)noexcept {
		externals.emplace(std::move(k), std::move(external(t...)));
	}
	auto equal_range(key k)noexcept {
		return externals.equal_range(k);
	}
	auto size()const noexcept { return externals.size(); }
private:
	//外力表 externals map
	std::multimap<key, external> externals;
};


template<typename constraint>
class tableConstraint {
public:
	tableConstraint() = default;
	~tableConstraint()noexcept = default;
	template<typename... T>void new_constraint(key k1, key k2, T &&... t)noexcept {
		constraints.emplace_back(k1, k2, std::move(t)...);
	}
	auto size()const noexcept { return constraints.size(); }
	const auto &operator[](size_t i)const noexcept {
		return this->constraints[i];
	}
private:
	struct con {
		template<typename... t>con(key a, key b, t &&...p) :a(a), b(b), data(std::move(p)...) { }
		~con()noexcept = default;
		key a, b;
		constraint data;
	};

	//约束表 constraints map
	std::vector<con> constraints;
};

template<typename shape>
class tableShape {
public:
	tableShape() = default;
	~tableShape()noexcept = default;
	template<typename... T>void new_shape(T &&... t)noexcept {
		shapes.emplace_back(std::move(t)...);
	}
	auto size()const noexcept { return shapes.size(); }
	const auto &operator[](size_t i)const noexcept {
		return this->shapes[i];
	}
private:
	//约束表 constraints map
	std::vector<shape> shapes;
};