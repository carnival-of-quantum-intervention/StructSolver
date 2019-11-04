#pragma once
#include <string>

class body {
public:
	const std::string name;
	body(const std::string &that)noexcept :name(that) { }
	body(const char *that)noexcept :name(that) { }
	~body() { }
};

class point {
public:
	const body &a, &b;

	point(const body &a, const body &b)noexcept :a(a), b(b) { }
};