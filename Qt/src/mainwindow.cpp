#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGridLayout>
#include <QFileDialog>
#include <QDir>

#include <osg/Group>
#include <osgDB/ReadFile>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, m_viewer_widget(Q_NULLPTR)
{
	ui->setupUi(this);

	QGridLayout *layout = new QGridLayout;
	m_viewer_widget = new QViewerWidget(QRect(0, 0, ui->frame->widthMM(), ui->frame->heightMM()));
	layout->addWidget(m_viewer_widget);
	ui->frame->setLayout(layout);
	ui->frame->setVisible(true);
	ui->verticalLayout->addWidget(ui->frame);

	ui->textEdit->setReadOnly(true);

	connect(&timer, &QTimer::timeout, this, &MainWindow::update);
	timer.start(40);

	connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::quit);
	connect(ui->actionClean, &QAction::triggered, this, &MainWindow::clean);
	//connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);

	initialize_folder();

	initialize_widget();

	this->setWindowTitle("3D industrial workpiece measurement system");
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::paintEvent(QPaintEvent *)
{
	ui->frame->update();
}

size_t MainWindow::write_log(const std::string log)
{
	std::string before_log = ">", postfix = "\n";

	ui->textEdit->moveCursor(QTextCursor::End);
	ui->textEdit->insertPlainText(QString::fromStdString(before_log + log + postfix));
	ui->textEdit->moveCursor(QTextCursor::End);

	return size_t(ui->textEdit->toPlainText().size());
}

void MainWindow::update()
{
	QMainWindow::update(this->geometry());
}

void MainWindow::clean()
{
	//    osg::Group *scene = qviewer->getScene();

	//    if (scene == nullptr)
	//        return;

	//    scene->removeChildren(0, scene->getNumChildren());
}

void MainWindow::quit()
{
	QApplication::quit();
}

void MainWindow::initialize_folder()
{
	m_export_folder = "output/";
	m_export_labeled_info_path = m_export_folder + "marked_points.txt";

	bool test;
	if (!QDir(m_export_folder).exists(m_export_folder))
	{
		test = QDir().mkdir(m_export_folder);
	}
}

void MainWindow::on_btn_start_labeling_clicked()
{
	m_viewer_widget->add_pick_handler();

	ui->btn_start_labeling->setDisabled(true);
	ui->btn_cancel_labeling->setDisabled(false);
}

void MainWindow::on_btn_cancel_labeling_clicked()
{
	m_viewer_widget->remove_pick_handler();
	m_viewer_widget->clear_point_cloud();

	m_viewer_widget->record_labeled_points();
	m_viewer_widget->clear_labeled_fitting();

	DETECT_TYPE dt;
	dt = m_viewer_widget->get_current_detection_type();

	std::string text = "recorded as ";
	if (dt == DETECT_TYPE::DT_POINT)
	{
		text += "Point";
	}
	else if (dt == DETECT_TYPE::DT_LINE)
	{
		text += "Line";
	}
	else if (dt == DETECT_TYPE::DT_PLANE)
	{
		text += "Plane";
	}
	else if (dt == DETECT_TYPE::DT_CYLINDER)
	{
		text += "Cylinder";
	}
	else
	{
		text += "Unknown"
	}
	write_log(text);

	ui->btn_start_labeling->setDisabled(false);
	ui->btn_cancel_labeling->setDisabled(true);
}

void MainWindow::on_btn_point_labeling_clicked()
{
	write_log("selected: point");

	m_viewer_widget->fit_picked_point_to_point();
}

void MainWindow::on_btn_line_labeling_clicked()
{
	write_log("selected: line");

	m_viewer_widget->fit_picked_point_to_line();
}

void MainWindow::on_btn_plane_labeling_clicked()
{
	write_log("selected: plane");
}

void MainWindow::on_btn_cylinder_labeling_clicked()
{
	write_log("selected: cylinder");
}


void MainWindow::initialize_widget()
{
	ui->btn_start_labeling->setDisabled(false);
	ui->btn_cancel_labeling->setDisabled(true);

	// initial 'show_axis' spindouble
	ui->doubleSpinBox_show_axis->setValue(1.0);
	ui->doubleSpinBox_show_axis->setRange(0.1, 20.0);
	ui->doubleSpinBox_show_axis->setSingleStep(0.1);
	if (ui->checkBox_show_axis->checkState() != Qt::Checked)
	{
		ui->doubleSpinBox_show_axis->setEnabled(false);
	}

	// initial 'point size' spindouble
    ui->spinBox_point_size->setValue(1);
    ui->spinBox_point_size->setMinimum(1);
    ui->spinBox_point_size->setSingleStep(1);

	// initial back and front color
	ui->pushButton_back_color->setStyleSheet("background: rgb(135,206,235)");
	ui->pushButton_front_color->setStyleSheet("background: rgb(0,0,0)");
    ui->pushButton_hover_color->setStyleSheet("background: rgb(255,255,255)");
    ui->pushButton_picked_color->setStyleSheet("background: rgb(255,0,0)");
    ui->pushButton_fitting_color->setStyleSheet("background: rgb(0,255,0)");

	m_viewer_widget->set_background_color(135, 206, 235, 1);
}

void MainWindow::on_btn_export_label_infor_clicked()
{
    std::map<std::string,std::vector<point_3d>> labeled_points_map;

    this->m_viewer_widget->get_labeled_points_map(labeled_points_map);

    std::map<std::string,std::vector<point_3d>>::iterator it;

    std::ofstream ofile(m_export_labeled_info_path.toStdString());
    if(!ofile.is_open()) return;

    for(it = labeled_points_map.begin();it != labeled_points_map.end();++it)
    {
        std::vector<point_3d> & points = it->second;
        for(size_t i=0; i<points.size(); ++i)
        {
            ofile << points[i].x << " "<< points[i].y << " "<< points[i].z ;
            ofile <<"\n";
        }
        ofile<<"#"<<it->first<<"\n";
    }
    ofile.close();

    write_log("wrote into: " + m_export_labeled_info_path.toStdString());
}

void MainWindow::on_pushButton_check_list_clicked()
{
	std::map<std::string, std::vector<point_3d>> labeled_points_map;

	this->m_viewer_widget->get_labeled_points_map(labeled_points_map);

	if (labeled_points_map.empty())
	{
		write_log("no labeled points");
	}

	std::map<std::string, std::vector<point_3d>>::iterator it;

	write_log("all labeling information are shown as below:");

	for (it = labeled_points_map.begin(); it != labeled_points_map.end(); ++it)
	{
		write_log(it->first + " : " + std::to_string(it->second.size()));
	}

	write_log("done");
}

void MainWindow::on_actionSettings_triggered()
{
    dialog_settings_ui = new dialog_settings(this);

	if (dialog_settings_ui->exec() == QDialog::Accepted)
	{
		dialog_settings_ui->get_configuration(m_str_str_map);
		dialog_settings_ui = nullptr;
	}
}

void MainWindow::on_btn_coarse_registration_clicked()
{
	QProcess *proc = new QProcess();

	QString program = "coarse_registration.exe";
	QString parameters = QString(m_str_str_map["configuration"].data());

	proc->start(program, QStringList() << parameters);
	proc->waitForFinished();
	QString result = proc->readAllStandardOutput();
	std::cout << result.toStdString();
	ui->textEdit->setText(result);

	show_warning("Coarse Reigstration", "Finished");
}

void MainWindow::on_btn_fine_registration_clicked()
{
	QProcess *proc = new QProcess();

	QString program = "fine_registration.exe";
	QString parameters = QString(m_str_str_map["configuration"].data());

	proc->start(program, QStringList() << parameters);
	proc->waitForFinished();
	QString result = proc->readAllStandardOutput();
	std::cout << result.toStdString();
	ui->textEdit->setText(result);
	show_warning("Fine Reigstration", "Finished");
}

void MainWindow::on_btn_visualization_clicked()
{
	QProcess *proc = new QProcess();

	QString program = "visualization_registration.exe";
	QString parameters = QString(m_str_str_map["configuration"].data());

	proc->startDetached(program, QStringList() << parameters, "./");
}

void MainWindow::on_pushButton_load_data_clicked()
{
    if (m_str_str_map.find("reference_data") == m_str_str_map.end())
    {
        show_warning("Warnning", "No reference data");
        return ;
    }

    std::string reference_data = m_str_str_map["reference_data"];

    if (ci.load_point_cloud_txt(reference_data, m_target_cloud_point))
    {
        write_log("read: " + reference_data + "(" + std::to_string(m_target_cloud_point.size()) + ")");
    }
    else
    {
        write_log("failed to read: " + reference_data);
    }
	m_viewer_widget->add_point_cloud(m_target_cloud_point, POINT_CLOUD, 1.0);
	m_viewer_widget->set_target_point_cloud(m_target_cloud_point);
}

void MainWindow::on_checkBox_show_axis_stateChanged(int arg1)
{
	if (this->ui->checkBox_show_axis->checkState() == Qt::Checked)
	{
		ui->doubleSpinBox_show_axis->setEnabled(true);
		this->m_viewer_widget->show_axis(float(ui->doubleSpinBox_show_axis->value()));
        write_log("open: show axis " + std::to_string(arg1));
	}
	else
	{
		ui->doubleSpinBox_show_axis->setEnabled(false);
		this->m_viewer_widget->hide_axis();
        write_log("close: show axis " + std::to_string(arg1));
	}
}

void MainWindow::on_spinBox_show_axis_valueChanged(const QString &arg1)
{
    on_checkBox_show_axis_stateChanged(arg1.toInt());
}

void MainWindow::on_spinBox_point_size_valueChanged(const QString &arg1)
{
    float point_size = float(ui->spinBox_point_size->value());
	this->m_viewer_widget->set_point_size(POINT_CLOUD, point_size);
    write_log("set point size: " + arg1.toStdString());
}

void MainWindow::on_pushButton_back_color_clicked()
{
	QColor color = QColorDialog::getColor(Qt::yellow, this);
	if (color.isValid())
	{
		float r, g, b;
		r = color.red();
		g = color.green();
		b = color.blue();
		this->m_viewer_widget->set_background_color(r, g, b, 1);

		QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
		ui->pushButton_back_color->setStyleSheet(html_content);

        write_log("set background color: " + html_content.toStdString());
	}

}

void MainWindow::on_pushButton_front_color_clicked()
{
	QColor color = QColorDialog::getColor(Qt::yellow, this);
	if (color.isValid())
	{
		float r, g, b;
		r = color.red();
		g = color.green();
		b = color.blue();
		this->m_viewer_widget->set_color(POINT_CLOUD, r, g, b, 1);

		QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
		ui->pushButton_front_color->setStyleSheet(html_content);

        write_log("set point cloud color: " + html_content.toStdString());
	}
	else
	{
		write_log("picked color is invalid");
	}
}

void MainWindow::on_pushButton_hover_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(HOVER_POINT, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_hover_color->setStyleSheet(html_content);

        write_log("set hover color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}

void MainWindow::on_pushButton_picked_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(PICKED_POINTS, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_picked_color->setStyleSheet(html_content);

        write_log("set picked color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}

void MainWindow::on_pushButton_fitting_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(FITTING_CLOUD, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_fitting_color->setStyleSheet(html_content);

        write_log("set fitting color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}
