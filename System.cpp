#include "main.h"


template<>
class table<Mode::system> {
	class external {
	public:
		constant x, y;
		constant fx, fy;
		external(const char *fx, const char *fy, const char *x, const char *y)
			:fx(fx), fy(fy), x(x), y(y) { }
		external(external &&that)noexcept
			:fx(std::move(that.fx)), fy(std::move(that.fy)), x(std::move(that.x)), y(std::move(that.y)) { }
		~external() { }

	private:

	};

	std::ostream &err;
	key countBody = 0;
	key countConstr = 0;
	//刚体表 rigid bodies map
	//基础（base）是具有不同性质的刚体 basement is a rigid body, though having different behaviours
	std::vector<std::string> bodies;
	//约束表 constraints map
	std::vector<constraint> constraints;
	//外力表
	std::multimap<key, external> externals;
public:
	//const body base = body("base", nullptr, nullptr);
	MY_LIB table(std::ostream &err)noexcept :err(err) { }
	MY_LIB table(const table &that) = default;

	auto new_external(key k, const char *fx, const char *fy, const char *x, const char *y)noexcept {
		return externals.emplace(k, std::move(external(fx, fy, x, y)));
	}
	key new_body(const char *name)noexcept {
		bodies.emplace_back(name);
		return countBody++;
	}

	void new_constraint(
		const std::string &a, const std::string &b,
		const std::string &x, const std::string &y,
		ang angle, bool force_or_moment
	)noexcept {
		try {
			key _a = key(-1), _b = key(-1);
			if (a != "base") {
				auto ia = std::find_if(bodies.cbegin(), bodies.cend(), [&a](const std::string &_body)->bool { return _body == a; });
				if (ia == bodies.cend()) {
					constexpr char tmp = '1';
					_a = new_body(a.c_str());
					err << "A body named " << a << " has been emplaced autimatically." << std::endl;
				}
				else _a = ia - bodies.cbegin();
			}
			if (b != "base") {
				auto ib = std::find_if(bodies.cbegin(), bodies.cend(), [&b](const std::string &_body)->bool { return _body == b; });
				if (ib == bodies.cend()) {
					constexpr char tmp = '1';
					_b = new_body(b.c_str());
					err << "A body named " << b << " has been emplaced autimatically." << std::endl;
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
		catch (const char *e) {
			err
				<< "Exception:\"" << e << "\"" << std::endl
				<< std::endl;
		}
	}
	void solve(std::ostream &out) noexcept {
		try {
			Math::NormalMatrix<constant> constr(countBody * 3, countConstr + 1);
			constr.fill([this](size_t i, size_t j) ->constant {
				try {
					//第(i / 3)个刚体，第(countConstr - j - 1)个约束
					//Please try Google Translation this time.
					if (j >= countConstr) {
						auto &&ext = externals.find(i / 3);
						if (ext == externals.cend()) {
							return constant(true, 0, 1);
						}
						else {
							switch (i % 3) {
							case 0:
								return (*ext).second.fx;
							case 1:
								return (*ext).second.fy;
							case 2:
								return (*ext).second.x * (*ext).second.fy - (*ext).second.y * (*ext).second.fx;
							default:
								return constant(true, 0, 1);
								break;
							}
						}
					}
					auto cons = constraints[j];
					//由于a b所受约束力方向相反，需要判断
					//As the direction of the constrain force to a and b is contrary, a variable should be used.
					bool is_a = true;
					if ((i / 3) == (cons).a || (is_a = false, (i / 3) == (cons).b)) {
						if ((cons).force_or_moment == true) {
							if (i % 3 == 0) {
								auto &&res = cosd(cons.direction);
								return  (is_a ? res : -res);
							}
							else if (i % 3 == 1) {
								auto &&res = sind(cons.direction);
								return  (is_a ? res : -res);
							}
							else if (i % 3 == 2) {
								auto &&res = (cons.x * sind(cons.direction) - cons.y * cosd(cons.direction));
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
			out << "Before solving:" << std::endl << std::setw(5) << constr << std::endl;
			auto rank = constr.to_diagon(countBody * 3, countConstr);
			out << "After solving:" << std::endl << std::setw(5) << constr << std::endl;
			if (rank < countBody * 3) {
				out
					<< "Mutable system! It is "
					<< (constr.no_more_than_one_nonzero_each_line() ? "variable" : "transient")
					<< " system. And it has "
					<< countBody * 3 - rank << " free degree. " << std::endl;
			}
			else if (countBody * 3 == countConstr) {
				out << "Statically determinate structure!" << std::endl;
			}
			else {
				out << "Statically indeterminate structure! Its degree of statical indeterminacy is " << countConstr - countBody * 3 << '.' << std::endl;
			}
		}
		catch (const char *e) {
			err
				<< "Exception:\"" << e << "\"" << std::endl
				<< std::endl;
		}
	}
private:
};


bool processSystem(std::istream &in, std::ostream &out, std::ostream &err)noexcept {
	using namespace LargeInteger;
	constexpr auto mode = Mode::system;

	table<mode> t(err);

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
				std::string name, fx, fy, xf, yf;


				ignore_if<' '>(in);
				getline<' ', '\n', '\r'>(in, name);

				char ch;
				while (ignore_if<')', ' '>(in), (ch = static_cast<char>(in.peek())) != '\n' && ch != '\r') {
					if (ignore_if_not<'(', '\n', '\r'>(in) == '(') {
						in.ignore();
					}

					switch (ch) {
					case 'f':
					{
						ignore_if<'(', ' '>(in);
						getline<',', '\n', '\r'>(in, xf);
						ignore_if<',', ' '>(in);
						getline<')', '\n', '\r'>(in, yf);

						ignore_if_not<'(', '\n', '\r'>(in);
						getline<',', '\n', '\r'>(in, fx);
						ignore_if<',', ' '>(in);
						getline<',', '\n', '\r'>(in, fy);

						break;
					}
					default:
						break;
					}
				}
				auto index = t.new_body(
					name.c_str()
				);
				if (!fx.empty() || !fy.empty()) {
					t.new_external(index, fx.c_str(), fy.c_str(), xf.c_str(), yf.c_str());
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