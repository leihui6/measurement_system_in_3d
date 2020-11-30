#include "common_use.h"

bool open_file(const std::string &file_name, std::fstream *f, bool clear)
{
    try
    {
        if (clear)
            f->open(file_name, std::fstream::in | std::fstream::out);
        else
            f->open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
    }
    catch (const std::exception&)
    {
        std::cerr << "[error]" << "file path:\"" << file_name << "\" does not exist.\n";
        f = nullptr;
        return false;
    }
    return f->is_open();
}

bool show_warning(std::string title, std::string text)
{
    QMessageBox msgBox;
    msgBox.setWindowTitle(QString(title.data()));
    msgBox.setText(QString(text.data()));
    return msgBox.exec();
}


bool read_file_as_map(const std::string &file_name, std::map<std::string, std::string> &str_str_map)
{
    std::fstream ifile;

    if (!open_file(file_name, &ifile))
    {
        show_warning("file warnning","No "+file_name);
        return false;
    }

    std::string line;

    while (std::getline(ifile, line))
    {
        if (line.empty()) continue;

        if (line[0] == '#') continue;

        size_t divided_flag = line.find(":");

        if (divided_flag == std::string::npos) continue;

        std::string
                key_ = line.substr(0, divided_flag++);

        std::string
                value_ = line.substr(divided_flag, line.size() - divided_flag);

        str_str_map[key_] = value_;
    }
    // std::cout << "read " << str_str_map.size() << " parameters from local file.\n";
    return true;
}
