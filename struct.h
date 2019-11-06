#pragma once
#include <string>
#include <forward_list>
#include <algorithm>
#include <map>
#include "D:\\MyLibrary\DefaultFunction.h"
#include "D:\\MyLibrary\NormalMatrix.h"

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
using key = size_t;

class constraint {
public:
	const key a, b;
	co x, y;
	bool force_or_moment;
	ang direction;
	constraint(key a, key b, co x, co y, ang angle, bool force_or_moment)noexcept
		:a(a), b(b), x(x), y(y), direction(angle), force_or_moment(force_or_moment) { }
};

class table {
	key countBody = 0;
	key countConstr = 0;
	//结点表 points map
	std::forward_list<constraint> constraints;
	//刚体表 bodies map
	//基础（base）是具有不同性质的刚体 basement is a body, though having different behaviours
	std::map<key, body> bodies;
public:
	const body base = body("base", nullptr, nullptr);
	__stdcall table() = default;
	__stdcall table(const table &that)noexcept = default;

	void emplace_body(const char *name, const char *x, const char *y)noexcept {
		try {
			auto &&fun_x = funEngine::produce(x), &&fun_y = funEngine::produce(y);
			bodies.emplace(countBody, body(name, std::move(fun_x), std::move(fun_y)));
		}
		catch (const std::exception & e) {
			std::cerr
				<< "Exception:\"" << e.what() << "\"" << std::endl
				<< "It may be caused by unsupported form of equation \"" << x
				<< "\" and \"" << y << "\"."
				<< std::endl;
		}
		++countBody;
	}
	void emplace_constraint(
		const std::string &a, const std::string &b,
		const std::string &x, const std::string &y,
		ang angle, bool force_or_moment
	)noexcept {
		constraints.emplace_front(
			(a == "base") ? key(0) : (*std::find_if(bodies.cbegin(), bodies.cend(), [&a](auto &_body) { return a == _body.second; })).first,
			(b == "base") ? key(0) : (*std::find_if(bodies.cbegin(), bodies.cend(), [&b](auto &_body) { return b == _body.second; })).first,
			std::atol(x.c_str()), std::atol(y.c_str()),
			angle, force_or_moment
		);
		++countConstr;
	}
	void solve()noexcept {
		Math::NormalMatrix<double> constr(countBody * 3, countConstr + 1);
		constr.fill([this](size_t i, size_t j) ->double {
			auto iter = constraints.cbegin();
			if (j >= countConstr)return 0;
			for (size_t t = 0; t < j; t++) {
				++iter;
			}
			bool is_a = true;
			if ((i / 3) == (*iter).a || (is_a = false, (i / 3) == (*iter).b)) {
				if ((*iter).force_or_moment == true) {
					if (i % 3 == 0) {
						return cos((*iter).direction * PI / 180) * (is_a ? 1 : -1);
					}
					if (i % 3 == 1) {
						return sin((*iter).direction * PI / 180) * (is_a ? 1 : -1);
					}
					if (i % 3 == 2) {
						return ((*iter).y * sin((*iter).direction * PI / 180) - (*iter).x * cos((*iter).direction * PI / 180)) * (is_a ? 1 : -1);
					}
					assert(false);
					return NAN;
				}
				else {
					if (i % 3 == 2) {
						return (is_a ? 1 : -1);
					}
					else return 0;
				}
			}
			else return 0;
			});
		std::cout << constr << std::endl;
		constr.to_diagon();
		std::cout << constr << std::endl;
	}
private:
};