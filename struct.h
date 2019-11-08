#pragma once
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include "D:\\MyLibrary\DefaultFunction.h"
#include "D:\\MyLibrary\NormalMatrix.h"

using namespace Darkness;

class body {
public:
	const std::string name;
	ptrHolder x, y;
	constant begin, end;
	body(const std::string &that, nullptr_t &&, nullptr_t &&, const char *begin, const char *end)noexcept
		:name(that), x(nullptr), y(nullptr), begin(begin), end(end) { }
	body(const std::string &that, ptrHolder &&x, ptrHolder &&y, const char *begin, const char *end)noexcept
		:name(that), x(std::move(x)), y(std::move(y)), begin(begin), end(end) { }
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
	std::vector<body> bodies;
	//约束表 constraints map
	std::vector<constraint> constraints;
public:
	//const body base = body("base", nullptr, nullptr);
	__stdcall table() = default;
	__stdcall table(const table &that) = default;


	auto emplace_body(const char *name, ptrHolder &&fun_x, ptrHolder &&fun_y, const char *begin, const char *end) {
		bodies.emplace_back(name, std::move(fun_x), std::move(fun_y), begin, end);
		return countBody++;
	}
	auto emplace_body(const char *name, nullptr_t, nullptr_t, const char *begin, const char *end) {
		bodies.emplace_back(name, nullptr, nullptr, begin, end);
		return countBody++;
	}

	void new_body(const char *name, const char *x, const char *y, const char* begin, const char* end)noexcept {
		try {
			ptrHolder &&fun_x = funEngine::produce(x), &&fun_y = funEngine::produce(y);
			if (!*x || !*y) {
				std::cout << "Shape of body \"" << name << "\" is not assigned." << std::endl;
			}
			else {
				if (!fun_x && *x) std::cerr << "Unsupported form of equation \"" << x << "\"." << std::endl;
				if (!fun_y && *y) std::cerr << "Unsupported form of equation \"" << y << "\"." << std::endl;
			}
			emplace_body(name, std::move(fun_x), std::move(fun_y), begin, end);
		}
		catch (const std::exception & e) {
			std::cerr << "Exception:\"" << e.what() << "\"" << std::endl;
		}
	}
	void new_constraint(
		const std::string &a, const std::string &b,
		const std::string &x, const std::string &y,
		ang angle, bool force_or_moment
	)noexcept {
		try {
			key _a = key(-1), _b = key(-1);
			if (a != "base") {
				auto ia = std::find_if(bodies.cbegin(), bodies.cend(), [&a](auto &_body) { return a == _body; });
				if (ia == bodies.cend()) {
					constexpr char tmp = '1';
					_a = emplace_body(a.c_str(), nullptr, nullptr, &tmp, &tmp);
					std::cerr << "A body named " << a << " has been emplaced autimatically. It has no shape asigned." << std::endl;
				}
				else _a = ia - bodies.cbegin();
			}
			if (b != "base") {
				auto ib = std::find_if(bodies.cbegin(), bodies.cend(), [&b](auto &_body) { return b == _body; });
				if (ib == bodies.cend()) {
					constexpr char tmp = '1';
					_b = emplace_body(b.c_str(), nullptr, nullptr, &tmp, &tmp);
					std::cerr << "A body named " << b << " has been emplaced autimatically. It has no shape asigned." << std::endl;
				}
				else _b = ib - bodies.cbegin();
			}
			constraints.emplace_back(
				_a, _b,
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
			std::cout << "Before solving:" << std::endl << std::setw(5) << constr << std::endl;
			auto rank = constr.to_diagon(countBody * 3, countConstr);
			std::cout << "After solving:" << std::endl << std::setw(5) << constr << std::endl;
			if (rank < countBody * 3) {
				std::cout
					<< "Mutable structure! It is "
					<< (constr.no_more_than_one_nonzero_each_line() ? "variable" : "transient")
					<< " structure. And it has "
					<< countBody * 3 - rank << " free degree. " << std::endl;
			}
			else if (countBody * 3 == countConstr) {
				std::cout << "Statically determinate structure!" << std::endl;
			}
			else {
				std::cout << "Statically indeterminate structure! Its degree of statical indeterminacy is " << countConstr - countBody * 3 << '.' << std::endl;
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

bool processInput(std::istream &stin)noexcept {
	using namespace LargeInteger;
	using namespace std;

	table t;

	string words;
	//per loop for per line
	do {
		if (!stin)return false;
		ignore_if<' '>(stin);
		auto c = getline<' ', '\n', '\r'>(stin, words);
		if (c == '\n' || c == '\r') {
			ignore_if<'\n', '\r'>(stin);
			continue;
		}

		if (words[0] != '/') {
			joint type = joint::unknown;
			if (words == "pole") {
				string name, x, y, begin, end, fx, fy;


				ignore_if<' '>(stin);
				getline<' ', '\n', '\r'>(stin, name);

				char ch;
				while (ignore_if<')', ' '>(stin), (ch = static_cast<char>(stin.peek())) != '\n' && ch != '\r') {
					if (ignore_if_not<'(', '\n', '\r'>(stin) == '(') {
						stin.ignore();
					}

					switch (ch) {
					case 'x':
					{
						getline<')', '\n', '\r'>(stin, x);
						break;
					}
					case 'y':
					{
						getline<')', '\n', '\r'>(stin, y);
						break;
					}
					case 't':
					{
						getline<',', '\n', '\r'>(stin, begin);
						getline<')', '\n', '\r'>(stin, end);
						break;
					}
					case 'f':
					{
						ignore_if_not<'(', '\n', '\r'>(stin);
						getline<',', '\n', '\r'>(stin, fx);
						ignore_if<',', ' '>(stin);
						getline<',', '\n', '\r'>(stin, fy);

						break;
					}
					default:
						break;
					}
				}

				t.new_body(
					name.c_str(),
					fx.c_str(), fy.c_str(),
					x.c_str(), y.c_str(),
					begin.c_str(), end.c_str()
				);
			}
			else if (
				(type = joint::lever, words == "lever")
				||
				(type = joint::pin, words == "pin" || words == "hinge")
				||
				(type = joint::rigid, words == "rigid")
				) {
				string a, b, x, y;
				ignore_if<' '>(stin);
				getline<' ', '\n', '\r'>(stin, a);
				ignore_if<' '>(stin);
				getline<' ', '\n', '\r'>(stin, b);
				ignore_if<' '>(stin);
				getline<' ', '\n', '\r'>(stin, x);
				ignore_if<' '>(stin);
				getline<' ', '\n', '\r'>(stin, y);
				switch (type) {
				case joint::lever:
				{
					string angle;
					ignore_if<' '>(stin);
					getline<' ', '\n', '\r'>(stin, angle);
					t.new_constraint(a, b, x, y, atol(angle.c_str()), true);
				}
				break;
				case joint::pin:
				{
					t.new_constraint(a, b, x, y, 0, true);
					t.new_constraint(a, b, x, y, 90, true);
				}
				break;
				case joint::rigid:
				{
					t.new_constraint(a, b, x, y, 0, true);
					t.new_constraint(a, b, x, y, 90, true);
					t.new_constraint(a, b, x, y, 90, false);
				}
				break;
				case joint::unknown:
				default:
					cerr << "Unknown joint type" << endl;
					break;
				}
			}
			else cerr << "Unknown input \"" << words << "\"." << endl;


			string res;
			ignore_if<' '>(stin);
			std::getline(stin, res);
			if (!res.empty()) cerr << "Unused input \"" << res << "\"." << endl;

		}
	} while (words.clear(), !stin.eof());

	t.solve();
	return true;
}