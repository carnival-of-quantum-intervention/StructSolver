#include <string>
#include <forward_list>
#include <iomanip>
#include "main.h"
#include "../../MyLibrary/CustomizedRadixCharSet.h"

template<Mode mode> bool processPathInput(const char *path, std::ostream &out, std::ostream &err)noexcept;
bool start(Mode mode, const char *path, std::ostream &out, std::ostream &err)noexcept;

//Caution:
//	every constraint should be single.
int main(int argc, char *argv[]) noexcept {
#ifdef _MSVC_LANG
	_CrtSetDbgFlag(_CRTDBG_CHECK_ALWAYS_DF | _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | _CRTDBG_CHECK_EVERY_16_DF);
#endif // _MSVC_LANG
	using namespace std;
	if (argc <= 1) {
		cerr << "No structure file input! Please input at least one in command line parameters." << endl;
		return -1;
	}
	else {
		Mode mode = Mode::unassigned;
		//per loop for per file
		for (int i = 1; i < argc; ++i) {
			if (*argv[i] == '?') {
				if (strcmp(argv[i] + 1, "struct") == 0) {
					mode = Mode::structure;
				}
				else if (strcmp(argv[i] + 1, "system") == 0) {
					mode = Mode::system;
				}
				continue;
			}
			start(mode, argv[i], cout, cerr);
		}
		return 0;
	}
}


template<Mode mode>
bool processPathInput(const char *path, std::ostream &out, std::ostream &err)noexcept {
	out << "Trying to open " << path << '.' << std::endl;
	std::ifstream fin(path);
	if (!fin)return err << "Error in opening " << path << '.' << std::endl, false;
	bool suc = true;
	try {
		switch (mode) {
		case Mode::system:
			suc = table<Mode::system>::process(fin, out, err);
			break;
		case Mode::structure:
			suc = table<Mode::structure>::process(fin, out, err);
			break;
		case Mode::unassigned:
		default:
			suc = false;
			break;
		}
	}
	catch (const char *e) {
		suc = false;
		err << "Exception:\"" << e << "\"" << std::endl;
	}
	if (!suc) {
		err << "Error in processing " << path << '.' << std::endl;
	}
	out << "Closing " << path << '.' << std::endl;
	fin.close();
	out << "Closed." << std::endl << std::endl << std::endl;
	return suc;
}

bool start(Mode mode, const char *path, std::ostream &out, std::ostream &err)noexcept {
	bool ret;
	switch (mode) {
	case Mode::system:
		ret = processPathInput<Mode::system>(path, out, err);
		break;
	case Mode::structure:
		ret = processPathInput<Mode::structure>(path, out, err);
		break;
	case Mode::unassigned:
	default:
		const char *begin = nullptr, *p = path;
		for (; *p != '\0'; p++) {
			if (*p == '.') begin = p;
		}
		if (begin) {
			if (strcmp(begin, ".struct") == 0) {
				ret = processPathInput<Mode::structure>(path, out, err);
				break;
			}
			else if (strcmp(begin, ".system") == 0) {
				ret = processPathInput<Mode::system>(path, out, err);
				break;
			}
		}
		//If not assigned, use system mode.
		//如果未指定，使用体系模式
		ret = processPathInput<Mode::system>(path, out, err);
		break;
	}
	return ret;
}