#include <iostream>
#include <string>

using namespace std;

string optimizePath(string seeked) {
    // LBR = B
    // LBS = R
    // RBL = B
    // SBL = R
    // SBS = B
    // LBL = S
    string opt; 
    do {
        for (int i = 0; i < seeked.length(); (i += 3)){
            string temp = to_string(seeked[i] + seeked[i+1] + seeked[i+2]);
            if      (temp == "LBR") opt += "B";
            else if (temp == "LBS") opt += "R";
            else if (temp == "RBL") opt += "B";
            else if (temp == "SBL") opt += "R";
            else if (temp == "SBS") opt += "B";
            else if (temp == "LBL") opt += "S";
            else opt += temp;
        }
        seeked = opt; 
    } while ((!opt.find("B")) && (opt.length() != 0));
    return opt; 
}

int main(){
    string a = "FFFFRLRFRFFLRFFFRFFFFRFRRLRRLFRLRFFFFFFLRLRRRLFFFFLFFLLFFRRFFRFRFFFFRRLRFFFLLFFLFFFFRFFRFFRFRRRRLRLFLLRLFLLRRFFFRLFRRLRRLLRRLLFFLRLFFLFL";
    string b = optimizePath(a);
    cout << a << endl << b << endl;
}
