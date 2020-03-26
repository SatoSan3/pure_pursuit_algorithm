#include "DataManager.hpp"
#include <fstream>

std::string DataManager::directoryName = "";

DataManager::DataManager()
{
}

DataManager::~DataManager()
{
}

void DataManager::storeData(std::vector<std::vector<dataType>> data)
{
    storedData.insert(storedData.end(), data.begin(), data.end());
}

void DataManager::storeSelectedData(std::vector<std::vector<dataType>> data)
{
    bool storeBool;
    for (int i = 0; i < data.size(); ++i) {
        storeBool = true;
        for (int j = 0; j < storedData.size(); ++j) {
            if (data[i][0] == storedData[j][0]) {
                storeBool = false;
                break;
            }
        }
        if (storeBool) {
            storedData.push_back(data[i]);
        }
    }
}

std::vector<std::vector<dataType>> DataManager::getStoredData()
{
    return storedData;
}

void DataManager::recordStoredData(std::string fileName)
{
    std::ofstream outputFile;
    //ディレクトリが新しく作成されていない場合
    if (directoryName == "") {
        outputFile = std::ofstream(fileName);
    } else {
#ifdef _WIN32
        outputFile = std::ofstream(directoryName + "\\" + fileName);
#elif __linux__
        outputFile = std::ofstream(directoryName + "/" + fileName);
#endif
    }

    for (int i = 0; i < storedData.size(); ++i) {
        for (int j = 0; j < storedData[i].size(); ++j) {
            outputFile << storedData[i][j];
            if (j < storedData[i].size() - 1) {
                outputFile << ",";
            }
        }
        outputFile << std::endl;
    }
    outputFile.close();
}

void DataManager::sortStoredData()
{
    std::sort(storedData.begin(), storedData.end(), [](const std::vector<dataType>& x, const std::vector<dataType>& y) { return x[0] > y[0]; });
}

void DataManager::readFileData(std::string fileName)
{
    std::ifstream ifs(fileName);

    std::string line;
    while (std::getline(ifs, line)) {

        std::vector<dataType> datavec = split(line, ',');
        storedData.push_back(datavec);
    }
}

void DataManager::readFileDataFromDirectory(std::string directory, std::string fileName)
{
#ifdef _WIN32
    readFileData(directory + "\\" + fileName);
#elif __linux__
    readFileData(directory + "/" + fileName);
#endif
}

bool DataManager::findData(dataType foundData, const int n)
{
    bool foundDataBool = false;
    for (int i = storedData.size() - 1; i >= 0; --i) {
        if ((storedData[i][n] - foundData) * (storedData[i][n] - foundData) < 12 * 12) {
            std::array<dataType, 3> route = {storedData[i][2], storedData[i][3], storedData[i][4]};
            //foundStoredData.insert(foundStoredData.begin(),route);
            foundStoredData.push_back(route);
            foundDataBool = true;
        } else {
            if (foundDataBool) {
                return true;
            }
        }
    }
    return false;
}

std::vector<std::array<dataType, 3>> DataManager::getFoundStoredData()
{
    return foundStoredData;
}

void DataManager::cleanFoundStoredData()
{
    std::vector<std::array<dataType, 3>>().swap(foundStoredData);
}

bool DataManager::makeDirectory(std::string newDirectoryName)
{
    if (tryMakeDirectory(newDirectoryName)) {
        directoryName = newDirectoryName;
        return true;
    }
    //無限ループ阻止（そもそも無限ループはしないと思うが..）
    for (int i = 0; i < 10000; i++) {
        std::string newDirectoryName2 = newDirectoryName + std::to_string(i);
        if (tryMakeDirectory(newDirectoryName2)) {
            directoryName = newDirectoryName2;
            return true;
        }
    }
    return false;
}
bool DataManager::tryMakeDirectory(std::string newDirectoryName)
{
    char* charDirectoryName = new char[newDirectoryName.size() + 1];
    newDirectoryName.copy(charDirectoryName, newDirectoryName.size());
    charDirectoryName[newDirectoryName.size()] = '\0';
    bool result = false;
#ifdef _WIN32
    //フォルダ作成でつかう
    if (_mkdir(charDirectoryName) == 0) {
#elif __linux__
    if (mkdir(charDirectoryName, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
#endif
        result = true;
    }
    delete[] charDirectoryName;
    return result;
}
std::vector<dataType> DataManager::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string n;
    std::vector<dataType> result;
    while (std::getline(stream, n, delimiter)) {
        if (typeid(dataType) == typeid(double)) {
            result.push_back(std::stod(n));
        } else if (typeid(dataType) == typeid(float)) {
            result.push_back(std::stof(n));
        } else if (typeid(dataType) == typeid(int)) {
            result.push_back(std::stoi(n));
        }
    }
    return result;
}
