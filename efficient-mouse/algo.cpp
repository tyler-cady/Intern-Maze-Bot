#include <iostream>
#include <string>

using namespace std;

string optimizePath(string seeked) {
    string opt;
    do {
        opt.clear();
        for (int i = 0; i < seeked.length(); i += 3) {
            string temp = seeked.substr(i, 3);
            if      (temp == "LBR") opt += "B";
            else if (temp == "LBF") opt += "R";
            else if (temp == "RBL") opt += "B";
            else if (temp == "FBL") opt += "R";
            else if (temp == "FBF") opt += "B";
            else if (temp == "LBL") opt += "F";
            else opt += temp;
        }
        seeked = opt;
        cout << seeked <<endl;
    } while (opt.find("B") != std::string::npos);
    return opt;
}

int main(){
    string a = "FFFFRLRFRFFLBRFFFRFFFFRFRRBLRRLFRLRFFFBFFFLRLRBRRLFFFFLFFLLFBFRRFFRFRFFFFRRLBRFFBFLLFFLFFBFFRFFRFFRFRRRRLRLFLLBRLFLLBRRFFFRLFRRLRRLLRRLLFFLRLFFLFL";
    string b = optimizePath(a);
    cout << a << endl << b << endl;
}
