#pragma once
#include <string>
#include <forward_list>
#include <algorithm>

class body {
public:
	const std::string name;
	body(const std::string &that)noexcept :name(that) { }
	body(const char *that)noexcept :name(that) { }
	~body() { }
	bool operator==(const std::string &str)noexcept { return this->name == str; }
};

enum class joint {
	lever, pin, rigid
};

class point {
public:
	const body &a, &b;
	const joint type;
	point(const body &a, const body &b, const joint& joint_type)noexcept :a(a), b(b),type(joint_type) { }
};

class table {
public:
	//���� points map
	std::forward_list<point> points;
	//����� bodies map
	//������base���Ǿ��в�ͬ���ʵĸ��� basement is a body, though having different behaviours
	std::forward_list<body> bodies;
	const body base = "base";

	auto emplace_front(const std::string &name)noexcept {
		return bodies.emplace_front(name);
	}
	auto emplace(const std::string &a, const std::string &b, const joint &joint_type)noexcept {
		const body
			&body1 = (a == "base") ? base : *std::find_if(bodies.begin(), bodies.end(), [&a](auto& _body) { return a == _body; }),
			&body2 = (b == "base") ? base : *std::find_if(bodies.begin(), bodies.end(), [&b](auto& _body) { return b == _body; });
		return points.emplace_front(body1, body2, joint_type);
	}

private:
};
