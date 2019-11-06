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
			table t;

			string words;
			//per loop for per line
			do {
				if (!fin) { cerr << "Error in reading " << argv[i] << '.' << endl; break; }
				ignore_if<' '>(fin);
				auto c = getline<' ', '\n', '\r'>(fin, words);
				if (c == '\n' || c == '\r')break;

				joint type = joint::unknown;
				if (words == "pole") {
					string name, x, y, begin, end;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, name);

					ignore_if_not<'x'>(fin);
					getline<'y', '\n', '\r'>(fin, x);

					ignore_if_not<'y'>(fin);
					getline<'('>(fin, y);

					ignore_if<'(', '=', ' ', ','>(fin);
					getline<',', ' ', '\n', '\r'>(fin, begin);
					getline<'\n', '\r'>(fin, end);


					t.emplace_body(name.c_str(), x.c_str(), y.c_str());
				}
				else if (
					(type = joint::lever, words == "lever")
					||
					(type = joint::pin, words == "pin" || words == "hinge")
					||
					(type = joint::rigid, words == "rigid")
					) {
					string a, b, x, y;
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, a);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, b);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, x);
					ignore_if<' '>(fin);
					getline<' ', '\n', '\r'>(fin, y);
					switch (type) {
					case joint::lever:
					{
						string angle;
						ignore_if<' '>(fin);
						getline<' ', '\n', '\r'>(fin, angle);
						t.emplace_constraint(a, b, x, y, atol(angle.c_str()), true);
					}
					break;
					case joint::pin:
					{
						t.emplace_constraint(a, b, x, y, 0, true);
						t.emplace_constraint(a, b, x, y, 90, true);
					}
					break;
					case joint::rigid:
					{
						t.emplace_constraint(a, b, x, y, 0, true);
						t.emplace_constraint(a, b, x, y, 90, true);
						t.emplace_constraint(a, b, x, y, 90, false);
					}
					break;
					case joint::unknown:
					default:
						cerr << "Unknown joint type" << endl;
						break;
					}
				}
				else cerr << "Unknown input \"" << words << "\"." << endl;


				string res;
				ignore_if<' '>(fin);
				std::getline(fin, res);
				if (!res.empty()) cerr << "Unused input \"" << res << "\"." << endl;

			} while (words.clear(), !fin.eof());

			t.solve();

			cout << "Closing " << argv[i] << '.' << endl;
			fin.close();
			cout << "Closed." << endl << endl << endl;
		}
		return 0;
	}
}