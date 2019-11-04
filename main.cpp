#include <iostream>
#include <fstream>
#include <string>
#include "D:\\MyLibrary\DefaultFunction.h"
#include "D:\\MyLibrary\CustomizedRadixCharSet.h"

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
			if (!fin)return cerr << "Error in opening " << argv[i] << '.' << endl, -1;
			if (fin.peek() == -1)return cerr << "Empty" << endl, -1;



			string words;
			//per loop for per line
			do {
				if (!fin) { cerr << "Error in reading " << argv[i] << '.' << endl; break; }
				ignore_if<' '>(fin);
				auto c = getline<' ', '\n', '\r'>(fin, words);
				ignore_if<' '>(fin);
				if (c == '\n' || c == '\r')break;
				if (words.empty()) continue;
				if (words == "point") {
				}
				else if (words == "body") {

				}
				else if (words=="pole") {

				}
				else if (words == "lever") {

				}
				else if (words == "pin" || words == "hinge") {

				}
				else if (words == "rigid") {

				}
				else cerr << "Unknown input \"" << words << "\"." << endl;
				string res;
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