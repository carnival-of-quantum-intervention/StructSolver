#include <iostream>
#include <fstream>
#include <sstream>
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
		for (int i = 1; i < argc; ++i) {
			cout << "Trying to open " << argv[i] << '.' << endl;
			ifstream fin(argv[i]);
			if (!fin)return cerr << "Error in opening " << argv[i] << '.' << endl, -1;
			if (fin.peek() == -1)return cerr << "Empty" << endl, -1;



			string words;
			do {
				if (!fin) { cerr << "Error in reading " << argv[i] << '.' << endl; break; }
				getline<' ', '\n', '\r'>(fin, words);
				if (words.empty())continue;
				if (words == "lever") {

				}
				else if (words == "pin" || words == "hinge") {

				}
				else if (words == "rigid") {

				}
				else cerr << "Unknown input \"" << words << "\"." << endl;
			} while (words.clear(), !fin.eof());
			cout << "Closing " << argv[i] << '.' << endl;
			fin.close();
			cout << "Closed." << endl << endl << endl;
		}
		return 0;
	}
}