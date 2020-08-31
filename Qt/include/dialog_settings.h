#ifndef DIALOG_SETTINGS_H
#define DIALOG_SETTINGS_H

#include <QDialog>

#include "common_use.h"

namespace Ui {
class dialog_settings;
}

class dialog_settings : public QDialog
{
    Q_OBJECT

public:
    explicit dialog_settings(QWidget *parent = nullptr);

    void get_configuration(std::map<std::string, std::string> & str_str_map);

    ~dialog_settings();

private slots:
    void on_btn_reload_clicked();

private:
    std::map<std::string, std::string> m_str_str_map;
    bool read_file_as_map(const std::string & file_name, std::map<std::string, std::string> & str_str_map);

private:
    std::string m_configuration;
    Ui::dialog_settings *ui;
};

#endif // DIALOG_SETTINGS_H
