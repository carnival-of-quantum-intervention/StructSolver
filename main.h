#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include "../../MyLibrary/DefaultFunction.h"
#include "../../MyLibrary/NormalMatrix.h"

using namespace Darkness;

using holder = std::unique_ptr<Expression>;

enum class Mode {
	unassigned, system, structure
};


using ang = size_t;//angle ½Ç¶È
using co = constant;//coordination ×ø±ê
using key = size_t;//¼üÖµ


enum class joint {
	unknown = -1, lever, pin, rigid
};

class constraint {
public:
	const key a, b;
	co x, y;
	bool force_or_moment;
	ang direction;
	constraint(key a, key b, co x, co y, ang angle, bool force_or_moment)noexcept
		:a(a), b(b), x(x), y(y), direction(angle), force_or_moment(force_or_moment) { }
};

template<Mode>class table;

bool processSystem(std::istream &, std::ostream &, std::ostream &) noexcept;
bool processStructure(std::istream &, std::ostream &, std::ostream &);