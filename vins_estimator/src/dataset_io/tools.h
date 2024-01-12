#ifndef TOOLS_H
#define TOOLS_H
namespace DATA_READER
{
void sortFileName(std::vector<std::string> &vFileName);
void checkLastCharForPath(std::string &dir);
int getFiles(std::string dir, std::vector<std::string> &vFileName);

int getFilesAllPath(std::string dir, std::vector<std::string> &vFileName);
int getImageFilesAllPath(std::string dir, std::vector<std::string> &vFileName);int getImageFilesAllPath(std::string dir, std::vector<std::string> &vFileName);
}
#endif