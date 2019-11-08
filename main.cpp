#include <iostream>
#include <fstream>
#include <string>
#include <forward_list>
#include <iomanip>
#include "struct.h"
#include "D:\\MyLibrary\CustomizedRadixCharSet.h"

//Caution:
//	every constraint should be single.
int main(int argc, char *argv[]) noexcept {
#ifdef _MSVC_LANG
	_CrtSetDbgFlag(_CRTDBG_CHECK_ALWAYS_DF | _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | _CRTDBG_CHECK_EVERY_16_DF);
#endif // _MSVC_LANG
	using namespace std;
	using namespace Function;
	using namespace LargeInteger;
	if (argc <= 1) {
		cerr << "No structure file input! Please input at least one in command line parameters." << endl;
		return -1;
	}
	else {
		cout << setprecision(15);
		//per loop for per file
		for (int i = 1; i < argc; ++i) {
			cout << "Trying to open " << argv[i] << '.' << endl;
			ifstream fin(argv[i]);
			(void)fin.peek();
			if (!fin)return cerr << "Error in opening " << argv[i] << '.' << endl, -1;

			if (!processInput(fin)) {
				cerr << "Error in processing " << argv[i] << '.' << endl;
			}
			cout << "Closing " << argv[i] << '.' << endl;
			fin.close();
			cout << "Closed." << endl << endl << endl;
		}
		return 0;
	}
}