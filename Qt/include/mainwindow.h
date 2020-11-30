#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QProcess>
#include <QColorDialog>
#include <QDebug>
#include <dialog_settings.h>

#include "common_use.h"
#include "cloud_io.h"
#include "qviewerwidget.h"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    dialog_settings * dialog_settings_ui;

    cloud_io ci;
    QTimer timer;
    QViewerWidget * m_viewer_widget;
    void paintEvent(QPaintEvent *);


    std::vector<point_3d> m_target_cloud_point;

    size_t write_log(const std::string log);

    bool initialize_parameters();

private slots:

    void update();

    void clean();

    void quit();

    void on_btn_start_labeling_clicked();

    void on_btn_cancel_labeling_clicked();

	void on_btn_point_labeling_clicked();

    void on_btn_line_labeling_clicked();

	void on_btn_plane_labeling_clicked();

	void on_btn_cylinder_labeling_clicked();

    void on_btn_export_label_infor_clicked();

    void on_actionSettings_triggered();

    void on_btn_coarse_registration_clicked();

    void on_btn_fine_registration_clicked();

    void on_btn_visualization_clicked();

    void on_pushButton_load_data_clicked();

    void on_checkBox_show_axis_stateChanged(int arg1);

    void on_doubleSpinBox_show_axis_valueChanged(const QString &arg1);

    void on_spinBox_point_size_valueChanged(const QString &arg1);

    void on_pushButton_back_color_clicked();

    void on_pushButton_front_color_clicked();

    void on_pushButton_check_list_clicked();

    void on_pushButton_hover_color_clicked();

    void on_pushButton_picked_color_clicked();

    void on_pushButton_fitting_color_clicked();

private:
    std::map<std::string, std::string> m_str_str_map;
    void initialize_widget();
	void initialize_folder();
    QString m_export_labeled_info_path;
	QString m_export_folder;
    std::string m_configuration_file;
};

#endif // MAINWINDOW_H
