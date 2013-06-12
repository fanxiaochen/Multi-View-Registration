#include <QDir>
#include <QSlider>
#include <QComboBox>
#include <QSettings>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QApplication>
#include <QKeyEvent>
#include <iostream>

#include "point_cloud.h"
#include "registrator.h"
#include "toggle_handler.h"
#include "parameter_manager.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"
#include "file_viewer_widget.h"
#include "main_window.h"

MainWindow::MainWindow(void)
  :registrator_(new Registrator),
  osg_viewer_widget_(NULL),
  file_viewer_widget_(NULL),
  workspace_(".")
{
  ui_.setupUi(this);

  MainWindowInstancer::getInstance().main_window_ = this;
  ParameterManager::getInstance();
  init();
}

MainWindow::~MainWindow()
{
  saveSettings();

  return;
}

void MainWindow::closeEvent(QCloseEvent *event)
{

  QMainWindow::closeEvent(event);

  return;
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
  FileSystemModel* model = getFileSystemModel();
  FileSystemModel::NavigationType type = FileSystemModel::SWITCH;
  type = (event->modifiers() == Qt::ControlModifier)?(FileSystemModel::APPEND):(type);
  type = (event->modifiers() == Qt::ShiftModifier)?(FileSystemModel::ERASE):(type);
  switch (event->key())
  {
  case (Qt::Key_Up):
    model->navigateToPreviousObject(type);
    break;
  case (Qt::Key_Down):
    model->navigateToNextObject(type);
    break;
  case (Qt::Key_Left):
    model->navigateToPreviousView(type);
    break;
  case (Qt::Key_Right):
    model->navigateToNextView(type);
    break;
  default:
    QMainWindow::keyPressEvent(event);
    break;
  }

  return;
}

MainWindow* MainWindow::getInstance()
{
  if (MainWindowInstancer::getInstance().main_window_ == NULL)
    std::cout << "shit happens!" << std::endl;
  return MainWindowInstancer::getInstance().main_window_;
}

FileSystemModel* MainWindow::getFileSystemModel(void)
{
  return file_viewer_widget_->getFileSystemModel();
}

void MainWindow::init(void)
{
  osg_viewer_widget_ = new OSGViewerWidget(this);
  file_viewer_widget_ = new FileViewerWidget(this);

  osg_viewer_widget_->addEventHandler(new ToggleHandler(registrator_, 'r', "Toggle Registrator."));
  osg_viewer_widget_->addChild(registrator_, false);

  setCentralWidget(osg_viewer_widget_);
  osg_viewer_widget_->startRendering();

  QDockWidget* dock_widget_status_file = new QDockWidget("File Viewer", this);
  addDockWidget(Qt::LeftDockWidgetArea, dock_widget_status_file);
  dock_widget_status_file->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
  file_viewer_widget_->setParent(dock_widget_status_file);
  dock_widget_status_file->setWidget(file_viewer_widget_);

  connect(this, SIGNAL(timeToUpdateStatusMessage(QString)), statusBar(), SLOT(showMessage(QString)));

  FileSystemModel* file_system_model = file_viewer_widget_->getFileSystemModel();
  //file menu
  connect(ui_.actionSetWorkspace, SIGNAL(triggered()), this, SLOT(setWorkspace()));
  createActionRecentWorkspaces();

  //rendering menu
  connect(ui_.actionIncreasePointSize, SIGNAL(triggered()), osg_viewer_widget_, SLOT(increasePointSize()));
  connect(ui_.actionDecreasePointSize, SIGNAL(triggered()), osg_viewer_widget_, SLOT(decreasePointSize()));
  connect(ui_.actionIncreaseLineWidth, SIGNAL(triggered()), osg_viewer_widget_, SLOT(increaseLineWidth()));
  connect(ui_.actionDecreaseLineWidth, SIGNAL(triggered()), osg_viewer_widget_, SLOT(decreaseLineWidth()));

  //registration menu
  connect(ui_.actionLoadAxis, SIGNAL(triggered()), registrator_, SLOT(load()));
  connect(ui_.actionSaveAxis, SIGNAL(triggered()), registrator_, SLOT(save()));
  connect(ui_.actionICP, SIGNAL(triggered()), registrator_, SLOT(registrationICP()));
  connect(ui_.actionRefineAxis, SIGNAL(triggered()), registrator_, SLOT(refineAxis()));
  connect(ui_.actionGenerateObject, SIGNAL(triggered()), registrator_, SLOT(registration()));
  connect(ui_.actionAutoReg, SIGNAL(triggered()), registrator_, SLOT(automaticRegistration()));

  loadSettings();

  return;
}

void MainWindow::openRecentWorkspace()
{
  QAction *action = qobject_cast<QAction *>(sender());
  if (action) {
    setWorkspace(action->data().toString());
  }

  return;
}

void MainWindow::createActionRecentWorkspaces()
{
  for (int i = 0; i < MaxRecentWorkspaces; ++i) {
    action_recent_workspaces_[i] = new QAction(this);
    action_recent_workspaces_[i]->setVisible(false);
    connect(action_recent_workspaces_[i], SIGNAL(triggered()), this, SLOT(openRecentWorkspace()));
  }

  for (size_t i = 0; i < MaxRecentWorkspaces; ++i) {
    ui_.menuRecentWorkspaces->addAction(action_recent_workspaces_[i]);
  }

  return;
}

void MainWindow::updateCurrentFile(const QString& filename)
{
  QFileInfo fileInfo(filename);
  setWindowTitle(tr("%1[*] - %2").arg(fileInfo.fileName()).arg(tr("Multi-View Registrator")));

  return;
}

void MainWindow::updateRecentWorkspaces()
{
  QMutableStringListIterator it(recent_workspaces_);
  while (it.hasNext()) {
    if (!QFile::exists(it.next()))
      it.remove();
  }

  recent_workspaces_.removeDuplicates();
  int num = (std::min)(MaxRecentWorkspaces, recent_workspaces_.size());
  for (int i = 0; i < num; ++i) {
    QString text = tr("&%1 %2").arg(i + 1).arg(recent_workspaces_[i]);
    action_recent_workspaces_[i]->setText(text);
    action_recent_workspaces_[i]->setData(recent_workspaces_[i]);
    action_recent_workspaces_[i]->setVisible(true);
  }

  for (int i = num; i < MaxRecentWorkspaces; ++ i) {
    action_recent_workspaces_[i]->setVisible(false);
  }

  while (recent_workspaces_.size() > num) {
    recent_workspaces_.removeAt(num);
  }

  return;
}

bool MainWindow::setWorkspace(void)
{                                
  QString directory;

  bool valid_workspace = false;
  do
  {
    directory = QFileDialog::getExistingDirectory(this,
      tr("Set Workspace"), workspace_,
      QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
      return false;


    if (directory.contains("object_"))
      valid_workspace = true;
    else if (directory.contains("points"))
      valid_workspace = true;
    else
    {
      QStringList entries = QDir(directory).entryList();
      bool has_points = false;
      for (QStringList::const_iterator it = entries.begin(); it != entries.end(); ++ it)
      {
        if (it->contains("points"))
          has_points = true;
        if (has_points)    
        {
          valid_workspace = true;
          break;
        }
      }
    }

  } while (!valid_workspace);


  setWorkspace(directory);

  return true;
}

void MainWindow::setWorkspace(const QString& workspace)
{
  workspace_ = workspace;
  recent_workspaces_.removeAll(workspace);
  recent_workspaces_.prepend(workspace);

  registrator_->reset();
  osg_viewer_widget_->removeChildren(true);
  file_viewer_widget_->setWorkspace(workspace_.toStdString());

  return;
}

void MainWindow::loadSettings()
{
  QSettings settings("MVR", "MVR");

  recent_workspaces_ = settings.value("recentWorkspaces").toStringList();
  updateRecentWorkspaces();

  workspace_ = settings.value("workspace").toString();

  setWorkspace(workspace_);

  return;
}

void MainWindow::saveSettings()
{
  QSettings settings("MVR", "MVR");
  settings.setValue("recentWorkspaces", recent_workspaces_);
  QString workspace(workspace_);
  settings.setValue("workspace", workspace);

  return;
}

void MainWindow::updateStatusMessage(const QString& message)
{
  emit timeToUpdateStatusMessage(message);

  return;
}

Messenger::Messenger(const QString& running_message, const QString& finished_message, QObject* parent)
  :QObject(parent), running_message_(running_message), finished_message_(finished_message)
{}

Messenger::~Messenger(void)
{}

void Messenger::sendRunningMessage(void)
{
  MainWindow::getInstance()->updateStatusMessage(running_message_);

  return;
}

void Messenger::sendFinishedMessage(void)
{
  MainWindow::getInstance()->updateStatusMessage(finished_message_);
  deleteLater();

  return;
}
