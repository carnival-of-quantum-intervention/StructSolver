#include "main.h"

template<>
class table<Mode::structure> {
	class body {
	public:
		const std::string name;
		holder x, y;
		constant begin, end;
		body(const std::string &name, nullptr_t &&, nullptr_t &&, const char *begin, const char *end)noexcept
			:name(name), x(nullptr), y(nullptr), begin(begin), end(end) { }
		body(const std::string &name, holder &&x, holder &&y, const char *begin, const char *end)noexcept
			:name(name), x(std::move(x)), y(std::move(y)), begin(begin), end(end) { }
		bool operator==(const std::string &str)const noexcept { return this->name == str; }
	};
	class external {
	public:
		constant t;
		constant fx, fy;
		external(const char *fx, const char *fy, const char *t)
			:fx(fx), fy(fy), t(t) { }
		external(external &&that)noexcept
			:fx(std::move(that.fx)), fy(std::move(that.fy)), t(std::move(that.t)) { }
		~external() { }

	private:

	};

	std::ostream &err;
	key countBody = 0;
	key countConstr = 0;
	//刚体表 rigid bodies map
	//基础（base）是具有不同性质的刚体 basement is a rigid body, though having different behaviours
	std::vector<body> bodies;
	//约束表 constraints map
	std::vector<constraint> constraints;
	//外力表
	std::multimap<key, external> externals;
public:
	//const body base = body("base", nullptr, nullptr);
	MY_LIB table(std::ostream &err)noexcept :err(err) { }
	MY_LIB table(const table &that) = default;


	auto emplace_body(const char *name, holder &&fun_x, holder &&fun_y, const char *begin, const char *end) {
		bodies.emplace_back(name, std::move(fun_x), std::move(fun_y), begin, end);
		return countBody++;
	}
	auto emplace_body(const char *name, nullptr_t, nullptr_t, const char *begin, const char *end) {
		bodies.emplace_back(name, nullptr, nullptr, begin, end);
		return countBody++;
	}
	auto new_external(key k, const char *fx, const char *fy, const char *t)noexcept {
		return externals.emplace(k, std::move(external(fx, fy, t)));
	}
	key new_body(const char *name, const char *x, const char *y, const char *begin, const char *end) {
		holder fun_x, fun_y;
		if (!*x || !*y) {
			err << "Shape of body \"" << name << "\" is not assigned." << std::endl;
			throw "Body shape not assigned.";
		}
		try {
			fun_x = funEngine::produce(x);
		}
		catch (const char *e) {
			err << "Exception:\"" << e << "\"" << std::endl << std::endl
				<< "Unsupported form of equation \"" << x << "\"." << std::endl;
			throw "Unsupported form of equation(In parameter 1).";
		}
		try {
			fun_y = funEngine::produce(y);
		}
		catch (const char *e) {
			err << "Exception:\"" << e << "\"" << std::endl << std::endl
				<< "Unsupported form of equation \"" << y << "\"." << std::endl;
			throw "Unsupported form of equation(In parameter 2).";
		}
		return emplace_body(name, std::move(fun_x), std::move(fun_y), begin, end);
	}

	void new_constraint(
		const std::string &a, const std::string &b,
		const std::string &x, const std::string &y,
		ang angle, bool force_or_moment
	) {
		try {
			key _a = key(-1), _b = key(-1);
			if (a != "base") {
				auto ia = std::find_if(bodies.cbegin(), bodies.cend(), [&a](const body &_body)->bool { return _body == a; });
				if (ia == bodies.cend()) {
					throw "Body name not defined(In parameter 1).";
				}
				else _a = ia - bodies.cbegin();
			}
			if (b != "base") {
				auto ib = std::find_if(bodies.cbegin(), bodies.cend(), [&b](const body &_body)->bool { return _body == b; });
				if (ib == bodies.cend()) {
					throw "Body name not defined(In parameter 2).";
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
			throw e;
		}
	}
	void solve(std::ostream &out) noexcept {
		try {
			Math::NormalMatrix<constant> constr(countBody * 3, countConstr + 1);
			constr.fill([this](const size_t &i, const size_t &j) ->constant {
				try {
					//第(i / 3)个刚体，第(countConstr - j - 1)个约束
					//Please try Google Translation this time.
					if (j >= countConstr) {
						auto &&exts = externals.equal_range(i / 3);
						constant res;
						for (auto ext = exts.first; ext != exts.second; ++ext) {
							switch (i % 3) {
							case 0:
								res += (*ext).second.fx;
							case 1:
								res += (*ext).second.fy;
							case 2:
								res += bodies[i / 3].x->getValue((*ext).second.t) * (*ext).second.fy - bodies[i / 3].y->getValue((*ext).second.t) * (*ext).second.fx;
							default:
								break;
							}
						}
						return res;
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
				return;
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

bool processStructure(std::istream &in, std::ostream &out, std::ostream &err) {
	using namespace LargeInteger;
	constexpr auto mode = Mode::structure;

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
			if (words == "straight") {
				std::string name, x, y, begin, end, fx, fy, para;


				ignore_if<' '>(in);
				getline<' ', '\n', '\r'>(in, name);

				char ch;
				while (in && (ignore_if<')', ' '>(in), (ch = static_cast<char>(in.peek())) != '\n' && ch != '\r')) {
					if (ignore_if_not<'(', '\n', '\r'>(in) == '(') {
						in.ignore();
					}

					switch (ch) {
					case 'x':
					{
						getline<')', '\n', '\r'>(in, x);
						break;
					}
					case 'y':
					{
						getline<')', '\n', '\r'>(in, y);
						break;
					}
					case 't':
					{
						getline<',', '\n', '\r'>(in, begin);
						getline<')', '\n', '\r'>(in, end);
						break;
					}
					case 'f':
					{
						ignore_if_not<'(', '\n', '\r'>(in);
						getline<',', '\n', '\r'>(in, fx);
						ignore_if<',', ' '>(in);
						getline<',', '\n', '\r'>(in, fy);

						break;
					}
					default:
						in.get();
						break;
					}
				}
				try {
					auto index = t.new_body(
						name.c_str(),
						x.c_str(), y.c_str(),
						begin.c_str(), end.c_str()
					);
					if (!fx.empty() || !fy.empty()) {
						t.new_external(index, fx.c_str(), fy.c_str(), para.c_str());
					}
				}
				catch (const char *e) {
					err << "Exception:\"" << e << "\"" << std::endl
						<< "Failed in create body." << std::endl
						<< "Info:\""
						<< name << "\", \""
						<< x << "\", \"" << y << "\", \""
						<< begin << "\", \"" << end << "\", \""
						<< fx << "\", \"" << fy << "\", \""
						<< para << "\"."
						<< std::endl;
					throw "illegal body.";
				}
			}
			else if (words == "pole") {
				std::string name, x, y, begin, end, fx, fy, para;


				ignore_if<' '>(in);
				getline<' ', '\n', '\r'>(in, name);

				char ch;
				while (in && (ignore_if<')', ' '>(in), (ch = static_cast<char>(in.peek())) != '\n' && ch != '\r')) {
					if (ignore_if_not<'(', '\n', '\r'>(in) == '(') {
						in.ignore();
					}

					switch (ch) {
					case 'x':
					{
						getline<')', '\n', '\r'>(in, x);
						break;
					}
					case 'y':
					{
						getline<')', '\n', '\r'>(in, y);
						break;
					}
					case 't':
					{
						getline<',', '\n', '\r'>(in, begin);
						getline<')', '\n', '\r'>(in, end);
						break;
					}
					case 'f':
					{
						ignore_if_not<'(', '\n', '\r'>(in);
						getline<',', '\n', '\r'>(in, fx);
						ignore_if<',', ' '>(in);
						getline<',', '\n', '\r'>(in, fy);

						break;
					}
					default:
						in.get();
						break;
					}
				}
				try {
					auto index = t.new_body(
						name.c_str(),
						x.c_str(), y.c_str(),
						begin.c_str(), end.c_str()
					);
					if (!fx.empty() || !fy.empty()) {
						t.new_external(index, fx.c_str(), fy.c_str(), para.c_str());
					}
				}
				catch (const char *e) {
					err << "Exception:\"" << e << "\"" << std::endl
						<< "Failed in create body." << std::endl
						<< "Info:\""
						<< name << "\", \""
						<< x << "\", \"" << y << "\", \""
						<< begin << "\", \"" << end << "\", \""
						<< fx << "\", \"" << fy << "\", \""
						<< para << "\"."
						<< std::endl;
					throw "illegal body.";
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
				try {
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
				catch (const char *e) {
					err
						<< "Exception:\"" << e << "\"" << std::endl
						<< "Info:" << a << ' ' << b << ' ' << x << ' ' << y << std::endl
						<< std::endl;
					throw "illegal constraint.";
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