#pragma once
#include <string>
#include <forward_list>
#include <algorithm>
#include <map>
#include "D:\\MyLibrary\DefaultFunction.h"
#include "D:\\MyLibrary\NormalMatrix.h"

using namespace Darkness;

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
using co = constant;
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
	//刚体表 rigid bodies map
	//基础（base）是具有不同性质的刚体 basement is a rigid body, though having different behaviours
	std::map<key, body> bodies;
	//约束表 constraints map
	std::forward_list<constraint> constraints;
public:
	//const body base = body("base", nullptr, nullptr);
	__stdcall table() = default;
	__stdcall table(const table &that) = default;

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
		try {
			constraints.emplace_front(
				(a == "base") ? key(-1) : (*std::find_if(bodies.cbegin(), bodies.cend(), [&a](auto &_body) { return a == _body.second; })).first,
				(b == "base") ? key(-1) : (*std::find_if(bodies.cbegin(), bodies.cend(), [&b](auto &_body) { return b == _body.second; })).first,
				x.c_str(), y.c_str(),
				angle, force_or_moment
			);
			++countConstr;
		}
		catch (const std::exception & e) {
			std::cerr
				<< "Exception:\"" << e.what() << "\"" << std::endl
				<< std::endl;
		}
	}
	void solve() noexcept {
		try {
			Math::NormalMatrix<constant> constr(countBody * 3, countConstr + 1);
			constr.fill([this](size_t i, size_t j) ->constant {
				try {
					//第(i / 3)个刚体，第(countConstr - j - 1)个约束
					//Please try Google Translation this time.
					auto iter = constraints.cbegin();
					if (j >= countConstr)return constant(true, 0, 1);
					for (size_t t = 0; t < j; t++) {
						++iter;
					}
					//由于a b所受约束力方向相反，需要判断
					//As the direction of the constrain force to a and b is contrary, a variable should be used.
					bool is_a = true;
					if ((i / 3) == (*iter).a || (is_a = false, (i / 3) == (*iter).b)) {
						if ((*iter).force_or_moment == true) {
							if (i % 3 == 0) {
								auto &&res = cosd((*iter).direction);
								return  (is_a ? res : -res);
							}
							else if (i % 3 == 1) {
								auto &&res = sind((*iter).direction);
								return  (is_a ? res : -res);
							}
							else if (i % 3 == 2) {
								auto &&res = ((*iter).x * sind((*iter).direction) - (*iter).y * cosd((*iter).direction));
								return (is_a ? res : -res);
							}
							assert(false);
							return constant(true, 0, 0);
						}
						else {
							if (i % 3 == 2) {
								return constant(is_a, 1, 1);
							}
							else return constant(true, 0, 1);
						}
					}
					else return constant(true, 0, 1);
				}
				catch (const std::exception & e) {
					std::cerr
						<< "Exception:\"" << e.what() << "\"" << std::endl
						<< "It occured at (" << i
						<< ", " << j << ")."
						<< std::endl;
					throw std::exception("Failed in filling the matrix.");
				}
				});
			std::cout << "Before solving:" << std::endl << constr << std::endl;
			auto rank = constr.to_diagon();
			std::cout << "After solving:" << std::endl << constr << std::endl;
			if (rank < countBody * 3) {
				std::cout << "Variable structure!It has " << countBody * 3 - rank << " free degree." << std::endl;
			}
			else if (countBody * 3 == countConstr) {
				std::cout << "Statically determinate structure!" << std::endl;
			}
			else {
				std::cout << "Statically indeterminate structure!Its degree of statical indeterminacy is " << countConstr - countBody * 3 << '.' << std::endl;
			}

		}
		catch (const std::exception & e) {
			std::cerr
				<< "Exception:\"" << e.what() << "\"" << std::endl
				<< std::endl;
		}
	}
private:
};