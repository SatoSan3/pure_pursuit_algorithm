#ifndef DATA_MANAGER_HPP
#define DATA_MANAGER_HPP

#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <typeinfo>
#include <array>

#ifdef _WIN32
//フォルダ作成でつかう
#include <direct.h>
#elif __linux__
#include <sys/stat.h>
#endif

//蓄積するデータの型を指定
typedef double dataType;
//typedef float dataType;
//typedef int dataType;
class DataManager
{
public:
    DataManager();
    ~DataManager();
    void storeData(std::vector<std::vector<dataType>> data);
    //storedData[i][0]が重複しないように格納する
    void storeSelectedData(std::vector<std::vector<dataType>> data);
    std::vector<std::vector<dataType>> getStoredData();
    void recordStoredData(std::string fileName);
    //storedData[i][0]のデータをソートする
    void sortStoredData();
    //ファイルからデータを読み込み、データを取り出す
    void readFileData(std::string fileName);
    void readFileDataFromDirectory(std::string directory, std::string fileName);
    bool findData(dataType foundData, const int n);
    std::vector<std::array<dataType, 3>> getFoundStoredData();
    void cleanFoundStoredData();
    bool makeDirectory(std::string newDirectoryName);

private:
    //データを保存するディレクトリは同じクラス同士で共有
    static std::string directoryName;
    std::vector<std::vector<dataType>> storedData;
    std::vector<std::array<dataType, 3>> foundStoredData;
    std::vector<dataType> split(std::string& input, char delimiter);
    bool tryMakeDirectory(std::string newDirectoryName);
};

#endif