#include "main.h"

inline namespace Structure {
	class body {
	public:
		holder x, y;
		constant begin, end;
		body(holder &&x, holder &&y, const char *begin, const char *end)noexcept
			:x(std::move(x)), y(std::move(y)), begin(begin), end(end) { }
	};
	class external {
	public:
		constant t;
		constant fx, fy;
		external(const char *fx, const char *fy, const char *t)
			:fx(fx), fy(fy), t(t) { }
		external(external &&that)noexcept
			:fx(std::move(that.fx)), fy(std::move(that.fy)), t(std::move(that.t)) { }
		~external()noexcept = default;
	};
	class constraint {
	public:
		co t1, t2;
		bool force_or_moment;
		ang direction;
		constraint(const char* t1,const char* t2, ang angle, bool force_or_moment)noexcept
			:t1(t1), t2(t2), direction(angle), force_or_moment(force_or_moment) { }
	};

	class tableStructure :protected tableName, tableExternal<external>, tableConstraint<constraint>, tableShape<body> {
		using names = tableName;
		using externals = tableExternal<external>;
		using constraints = tableConstraint<constraint>;
		using shapes = tableShape<body>;
	public:
		//const body base = body("base", nullptr, nullptr);
		tableStructure(std::ostream &err)noexcept :names(err) { }
		tableStructure(const tableStructure &that) = default;

		auto new_external(key k, const char *fx, const char *fy, const char *t)noexcept {
			return externals::new_external(k, std::move(external(fx, fy, t)));
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
			auto key = names::new_name(name);
			if (key == names::size() - 1) {
				shapes::new_shape(std::move(fun_x), std::move(fun_y), begin, end);
				assert(names::size() == shapes::size());
			}
			else {
				err << "Repeated name!" << std::endl;
			}
		}

		void new_constraint(
			const std::string &a, const std::string &b,
			const std::string &t1, const std::string &t2,
			ang angle, bool force_or_moment
		) {
			try {
				key _a = key(-1), _b = key(-1);
				if (a != "base") {
					auto ia = std::find_if(names::cbegin(), names::cend(), [&a](const std::string &_body)->bool { return _body == a; });
					if (ia == names::cend()) {
						throw "Body name not defined(In parameter 1).";
					}
					else _a = ia - names::cbegin();
				}
				if (b != "base") {
					auto ib = std::find_if(names::cbegin(), names::cend(), [&b](const std::string &_body)->bool { return _body == b; });
					if (ib == names::cend()) {
						throw "Body name not defined(In parameter 2).";
					}
					else _b = ib - names::cbegin();
				}
				constraints::new_constraint(
					_a, _b,
					t1.c_str(), t2.c_str(),
					angle, force_or_moment
				);
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
				Math::NormalMatrix<constant> constr(names::size() * 3, constraints::size() + 1);
				constr.fill([this](const size_t &i, const size_t &j) ->constant {
					try {
						//第(i / 3)个刚体，第(countConstr - j - 1)个约束
						//Please try Google Translation this time.
						if (j >= constraints::size()) {
							auto &&exts = externals::equal_range(i / 3);
							constant res;
							for (auto ext = exts.first; ext != exts.second; ++ext) {
								switch (i % 3) {
								case 0:
									res += (*ext).second.fx;
								case 1:
									res += (*ext).second.fy;
								case 2:
									res += this->shapes::operator[](i / 3).x->getValue((*ext).second.t) * (*ext).second.fy - this->shapes::operator[](i / 3).y->getValue((*ext).second.t) * (*ext).second.fx;
								default:
									break;
								}
							}
							return res;
						}
						auto cons = this->constraints::operator[](j);
						//由于a b所受约束力方向相反，需要判断
						//As the direction of the constrain force to a and b is contrary, a variable should be used.
						bool is_a = true;
						if ((i / 3) == (cons).a || (is_a = false, (i / 3) == (cons).b)) {
							if ((cons).data.force_or_moment == true) {
								const auto &t = (is_a) ? cons.a : cons.b;
								if (i % 3 == 0) {
									auto &&res = cosd(cons.data.direction);
									return  (is_a ? res : -res);
								}
								else if (i % 3 == 1) {
									auto &&res = sind(cons.data.direction);
									return  (is_a ? res : -res);
								}
								else if (i % 3 == 2) {
									auto &&res = (this->shapes::operator[](j).x->getValue(t) * sind(cons.data.direction) - this->shapes::operator[](j).y->getValue(t) * cosd(cons.data.direction));
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
				auto rank = constr.to_diagon(names::size() * 3, constraints::size());
				out << "After solving:" << std::endl << std::setw(5) << constr << std::endl;
				if (rank < names::size() * 3) {
					out
						<< "Mutable system! It is "
						<< (constr.no_more_than_one_nonzero_each_line() ? "variable" : "transient")
						<< " system. And it has "
						<< names::size() * 3 - rank << " free degree. " << std::endl;
					return;
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


		static bool process(std::istream &in, std::ostream &out, std::ostream &err) {
			using namespace LargeInteger;
			constexpr auto mode = Mode::structure;

			tableStructure t(err);

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
							if (ignore_if<' ', 'x', 'y', 'z', 't'>(in) == '(') {
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
								ignore_if<'f', ' '>(in);
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
							if (ignore_if<' ', 'x', 'y', 'z', 't'>(in) == '(') {
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
								ignore_if<'f', ' '>(in);
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
	private:
	};
}