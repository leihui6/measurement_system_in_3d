#include "dialog_settings.h"
#include "ui_dialog_settings.h"

dialog_settings::dialog_settings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dialog_settings)
{
    ui->setupUi(this);

    QString default_config_path = "data/configuration.txt";
    this->ui->lineEdit_configuration->setText(default_config_path);

    on_btn_reload_clicked();
}

void dialog_settings::get_configuration(std::map<std::string, std::string> &str_str_map)
{
    str_str_map = m_str_str_map;
}

dialog_settings::~dialog_settings()
{
    delete ui;
}

void dialog_settings::on_btn_reload_clicked()
{
    m_configuration = ui->lineEdit_configuration->text().toStdString();

    m_str_str_map["configuration"] = m_configuration;

    if (!read_file_as_map(m_configuration, m_str_str_map)) return;

    ui->lineEdit_reading_data->setText(QString(m_str_str_map["reading_data"].data()));
    ui->lineEdit_reference_data->setText(QString(m_str_str_map["reference_data"].data()));
    ui->lineEdit_output_folder->setText(QString(m_str_str_map["output_folder"].data()));
    ui->lineEdit_label_filename->setText(QString(m_str_str_map["label_filename"].data()));
    ui->lineEdit_icp_configuration->setText(QString(m_str_str_map["icp_configuration"].data()));
    ui->lineEdit_4pcs_configuration->setText(QString(m_str_str_map["4pcs_configuration"].data()));
    ui->lineEdit_measurement_filename->setText(QString(m_str_str_map["measurement_filename"].data()));
    ui->lineEdit_coarse_registration_filename->setText(QString(m_str_str_map["coarse_registration_filename"].data()));
    ui->lineEdit_fine_registration_filename->setText(QString(m_str_str_map["fine_registration_filename"].data()));
}
