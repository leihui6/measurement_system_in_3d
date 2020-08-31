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
