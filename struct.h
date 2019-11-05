#pragma once
#include <string>
#include <forward_list>
#include <algorithm>
#include "D:\\MyLibrary\DefaultFunction.h"

class body {
public:
	const std::string name;
	ptrHolder x, y;
	body(const std::string &that, function *x, function *y)noexcept :name(that), x(x), y(y) { }
	body(const std::string &that, ptrHolder &&x, ptrHolder &&y)noexcept
		:name(that), x(std::move(x)), y(std::move(y)) { }
	body(const char *that, function *x, function *y)noexcept :name(that), x(x), y(y) { }
	bool operator==(const std::string &str)const noexcept { return this->name == str; }
};

enum class joint {
	unknown = -1, lever, pin, rigid
};

using ang = size_t;
using co = size_t;

class constraint {
public:
	const body &a, &b;
	co x, y;
	bool force_or_moment;
	ang direction;
	constraint(const body &a, const body &b, co x, co y, ang angle, bool force_or_moment)noexcept
		:a(a), b(b), x(x), y(y), direction(angle), force_or_moment(force_or_moment) { }
};

class table {
public:
	//结点表 points map
	std::forward_list<constraint> constraints;
	//刚体表 bodies map
	//基础（base）是具有不同性质的刚体 basement is a body, though having different behaviours
	std::forward_list<body> bodies;
	const body base = body("base", nullptr, nullptr);

	void emplace_body(const char*name, const char* x, const char* y)noexcept {
		try {
			auto &&fun_x = funEngine::produce(x), &&fun_y = funEngine::produce(y);
			bodies.emplace_front(name, std::move(fun_x), std::move(fun_y));
		}
		catch (const std::exception & e) {
			std::cerr
				<< "Exception:\"" << e.what() << "\"" << std::endl
				<< "It may be caused by unsupported form of equation \"" << x
				<< "\" and \"" << y << "\"."
				<< std::endl;
		}
	}
	void emplace_constraint(
		const std::string &a, const std::string &b,
		const std::string &x, const std::string &y,
		ang angle, bool force_or_moment
	)noexcept {
		const body
			&body1 = (a == "base") ? base : *std::find_if(bodies.cbegin(), bodies.cend(), [&a](auto &_body) { return a == _body; }),
			&body2 = (b == "base") ? base : *std::find_if(bodies.cbegin(), bodies.cend(), [&b](auto &_body) { return b == _body; });
		constraints.emplace_front(body1, body2, std::atol(x.c_str()), std::atol(y.c_str()), angle, force_or_moment);
	}

private:
};
