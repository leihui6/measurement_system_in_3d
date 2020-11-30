#ifndef COMMON_USE_H
#define COMMON_USE_H

#include <QString>
#include <QMessageBox>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

bool open_file(const std::string & file_name, std::fstream * f, bool clear = false);

bool show_warning(std::string title, std::string text);

bool read_file_as_map(const std::string &file_name, std::map<std::string, std::string> &str_str_map);

#endif // COMMON_USE_H

