#include <iostream>
#include <fstream>
#include <string>
#include <forward_list>
#include <iomanip>
#include "struct.h"
#include "..\..\MyLibrary\CustomizedRadixCharSet.h"

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