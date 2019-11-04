#include <iostream>
#include <fstream>
#include <string>
#include <forward_list>
#include "struct.h"
#include "D:\\MyLibrary\DefaultFunction.h"
#include "D:\\MyLibrary\CustomizedRadixCharSet.h"

//Caution:
//	every constraint should be single.
int main(int argc, char *argv[]) noexcept {
	using namespace std;
	using namespace Function;
	using namespace LargeInteger;
	if (argc <= 1) {
		cerr << "No structure file input! Please input at least one in command line parameters." << endl;
		return -1;
	}
	else {
		cout << "Here is " << argv[0] << endl;
		//per loop for per file
		for (int i = 1; i < argc; ++i) {
			cout << "Trying to open " << argv[i] << '.' << endl;
			ifstream fin(argv[i]);
			(void)fin.peek();
			if (!fin)return cerr << "Error in opening " << argv[i] << '.' << endl, -1;

			string words;
			//per loop for per line
			do {
				if (!fin) { cerr << "Error in reading " << argv[i] << '.' << endl; break; }
				ignore_if<' '>(fin);
				auto c = getline<' ', '\n', '\r'>(fin, words);
				if (c == '\n' || c == '\r')break;

				table t;

				if (words == "pole") {
					string name;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, name);
					t.bodies.emplace_front(name);
				}
				else if (words == "lever") {
					string a, b;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, a);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, b);
					t.points.emplace_front(a, b, joint::lever);
				}
				else if (words == "pin" || words == "hinge") {
					string a, b;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, a);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, b);
					t.points.emplace_front(a, b, joint::pin);
				}
				else if (words == "rigid") {
					string a, b;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, a);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, b);
					t.points.emplace_front(a, b, joint::rigid);
				}
				else cerr << "Unknown input \"" << words << "\"." << endl;


				string res;
				ignore_if<' '>(fin);
				std::getline(fin, res);
				if (!res.empty()) cerr << "Unused input \"" << res << "\"." << endl;

			} while (words.clear(), !fin.eof());
			cout << "Closing " << argv[i] << '.' << endl;
			fin.close();
			cout << "Closed." << endl << endl << endl;
		}
		return 0;
	}
}