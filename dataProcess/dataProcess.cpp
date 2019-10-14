//
// Created by jixingwu on 2019/10/14.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;


using namespace std;
//删除字符串中空格，制表符tab等无效字符
string Trim(string& str) {
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

int main(int argc, char** argv){
    ifstream inFile(argv[1]. ios::in);
    vector<double>









    return 0;

}

