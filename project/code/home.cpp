#include "home.hpp"

using namespace std;
#include <fstream>

int readFlag(const std::string& filename)
{
    std::ifstream file(filename);
    int flag = 0;
    if (file.is_open()) {
        file >> flag; // 读取文件中的更新标志
        file.close();
    } else {
        std::cerr << "Failed to open " << filename << std::endl;
    }
    return flag;
}
