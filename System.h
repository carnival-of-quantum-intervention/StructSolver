#pragma once
#include "main.h"

inline namespace System {
	class external {
	public:
		constant x, y;
		constant fx, fy;
		external(const char *fx, const char *fy, const char *x, const char *y)
			:fx(fx), fy(fy), x(x), y(y) { }
		external(const char *&&fx, const char *&&fy, const char *&&x, const char *&&y)
			:fx(fx), fy(fy), x(x), y(y) { }
		external(external &&that)noexcept
			:fx(std::move(that.fx)), fy(std::move(that.fy)), x(std::move(that.x)), y(std::move(that.y)) { }
		~external() { }
	private:
	};
	class constraint {
	public:
		co x, y;
		bool force_or_moment;
		ang direction;
		constraint(const char *x, const char *y, ang angle, bool force_or_moment)noexcept
			:x(x), y(y), direction(angle), force_or_moment(force_or_moment) { }
	};

	class tableSystem :protected tableName, tableExternal<external>, tableConstraint<constraint> {
		using names = tableName;
		using externals = tableExternal<external>;
		using constraints = tableConstraint<constraint>;
	public:
		tableSystem(std::ostream &err)noexcept :names(err) { }
		tableSystem(const tableSystem &that) = default;

		void new_constraint(
			const std::string &a, const std::string &b,
			const std::string &x, const std::string &y,
			ang angle, bool force_or_moment
		)noexcept {
			try {
				key _a = key(-1), _b = key(-1);
				if (a != "base") {
					auto ia = std::find_if(names::cbegin(), names::cend(), [&a](const std::string &_body)->bool { return _body == a; });
					if (ia == names::cend()) {
						constexpr char tmp = '1';
						_a = new_name(a.c_str());
						err << "A body named " << a << " has been emplaced autimatically." << std::endl;
					}
					else _a = ia - names::cbegin();
				}
				if (b != "base") {
					auto ib = std::find_if(names::cbegin(), names::cend(), [&b](const std::string &_body)->bool { return _body == b; });
					if (ib == names::cend()) {
						constexpr char tmp = '1';
						_b = new_name(b.c_str());
						err << "A body named " << b << " has been emplaced autimatically." << std::endl;
					}
					else _b = ib - names::cbegin();
				}
				this->constraints::new_constraint(
					_a, _b,
					x.c_str(), y.c_str(),
					angle, force_or_moment
				);
			}
			catch (const char *e) {
				err
					<< "Exception:\"" << e << "\"" << std::endl
					<< std::endl;
			}
		}
		void solve(std::ostream &out) noexcept {
			try {
				Math::NormalMatrix<constant> constr(this->names::size() * 3, this->constraints::size());
				constr.fill([this](size_t i, size_t j) ->constant {
					try {
						auto cons = this->constraints::operator[](j);
						//由于a b所受约束力方向相反，需要判断
						//As the direction of the constrain force to a and b is contrary, a variable should be used.
						bool is_a = true;
						if ((i / 3) == cons.a || (is_a = false, (i / 3) == cons.b)) {
							if (cons.data.force_or_moment == true) {
								if (i % 3 == 0) {
									auto &&res = cosd(cons.data.direction);
									return  (is_a ? res : -res);
								}
								else if (i % 3 == 1) {
									auto &&res = sind(cons.data.direction);
									return  (is_a ? res : -res);
								}
								else if (i % 3 == 2) {
									auto &&res = (cons.data.x * sind(cons.data.direction) - cons.data.y * cosd(cons.data.direction));
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
					catch (const char *e) {
						err
							<< "Exception:\"" << e << "\"" << std::endl
							<< "It occured at (" << i
							<< ", " << j << ")."
							<< std::endl;
						throw "Failed in filling the matrix.";
					}
					});
				out << "Before solving:" << std::endl << std::setw(6) << constr << std::endl;
				auto rank = constr.to_diagon(names::size() * 3, constraints::size());
				out << "After solving:" << std::endl << std::setw(5) << constr << std::endl;
				if (rank < names::size() * 3) {
					out
						<< "Mutable system! It is "
						<< (constr.no_more_than_one_nonzero_each_line() ? "variable" : "transient")
						<< " system. And it has "
						<< names::size() * 3 - rank << " free degree. " << std::endl;
				}
				else if (names::size() * 3 == constraints::size()) {
					out << "Statically determinate structure!" << std::endl;
				}
				else {
					out << "Statically indeterminate structure! Its degree of statical indeterminacy is " << constraints::size() - names::size() * 3 << '.' << std::endl;
				}
			}
			catch (const char *e) {
				err
					<< "Exception:\"" << e << "\"" << std::endl
					<< std::endl;
			}
		}
		static bool process(std::istream &in, std::ostream &out, std::ostream &err)noexcept {
			using namespace LargeInteger;
			constexpr auto mode = Mode::system;

			tableSystem t(err);

			std::string words;
			//per loop for per line
			do {
				if (!in)return false;
				ignore_if<' '>(in);
				auto c = getline<' ', '\n', '\r'>(in, words);
				if (c == '\n' || c == '\r') {
					ignore_if<'\n', '\r'>(in);
					continue;
				}

				if (words[0] != '/') {
					joint type = joint::unknown;
					if (words == "pole") {
						std::string name;

						ignore_if<' '>(in);
						getline<' ', '\n', '\r'>(in, name);
						auto key = t.new_name(name.c_str());
						while (!BaseSet<char, char, '\n', '\r'>::exist(ignore_if < ' ' >(in))) {
							if (in.peek() == 'f') {
								std::string x, y, fx, fy;
								ignore_if<' ', 'f', '('>(in);
								getline<',', '\n', '\r'>(in, x);
								getline<')', '\n', '\r'>(in, y);
								ignore_if<' ', '=', '('>(in);
								getline<',', '\n', '\r'>(in, fx);
								getline<')', '\n', '\r'>(in, fy);
								t.externals::new_external(key, fx.c_str(), fy.c_str(), x.c_str(), y.c_str());
							}
							else break;
						}
					}
					else if (
						(type = joint::lever, words == "lever")
						||
						(type = joint::pin, words == "pin" || words == "hinge")
						||
						(type = joint::rigid, words == "rigid")
						) {
						std::string a, b, x, y;
						ignore_if<' '>(in);
						getline<' ', '\n', '\r'>(in, a);
						ignore_if<' '>(in);
						getline<' ', '\n', '\r'>(in, b);
						ignore_if<' '>(in);
						getline<' ', '\n', '\r'>(in, x);
						ignore_if<' '>(in);
						getline<' ', '\n', '\r'>(in, y);
						switch (type) {
						case joint::lever:
						{
							std::string angle;
							ignore_if<' '>(in);
							getline<' ', '\n', '\r'>(in, angle);
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
							err << "Unknown joint type" << std::endl;
							break;
						}
					}
					else err << "Unknown input \"" << words << "\"." << std::endl;


					std::string res;
					ignore_if<' '>(in);
					std::getline(in, res);
					if (!res.empty()) err << "Unused input \"" << res << "\"." << std::endl;

				}
			} while (words.clear(), !in.eof());

			t.solve(out);
			return true;
		}
	private:
	};
}