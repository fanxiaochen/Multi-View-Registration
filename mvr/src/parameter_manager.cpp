#include <iostream>
#include <QDomElement>
#include <QDomDocument>
#include <QTextStream>

#include "parameter.h"
#include "main_window.h"
#include "parameter_dialog.h"
#include "file_system_model.h"
#include "parameter_manager.h"

ParameterManager::ParameterManager(void)
  :registration_max_iterations_(new IntParameter("Max Iterations", "Max Iterations", 64, 1, 1024)),
  registration_max_distance_(new DoubleParameter("Max Distance", "Max Distance", 4, 1, 16, 1.0)),
  start_object_(new IntParameter("Start object", "Start object", -1, -1, -1, 1)),
  end_object_(new IntParameter("End object", "End object", -1, -1, -1, 1)),
  current_object_(new IntParameter("Current object", "Current object", -1, -1, -1, 1)),
  repeat_times_(new IntParameter("Repeat times", "Repeat times", 5, 1, 100, 1)),
  triangle_length_(new DoubleParameter("Triangle Length", "Triangle Length", 2.5, 1.0, 8.0, 0.1)),
  segment_threshold_(new IntParameter("Segment Threshold", "Segment Threshold", 10, 10, 500, 10))
{
}

ParameterManager::~ParameterManager(void)
{

  delete registration_max_distance_;
  delete registration_max_iterations_;
  delete start_object_;
  delete end_object_;
}

void ParameterManager::initObjectNumbers(void)
{
  int start_object, end_object;
  MainWindow::getInstance()->getFileSystemModel()->getObjectRange(start_object, end_object);

  start_object_->setDefaultValue(start_object);
  start_object_->setValue(start_object);
  start_object_->setLow(start_object);
  start_object_->setHigh(end_object);

  end_object_->setDefaultValue(end_object);
  end_object_->setValue(end_object);
  end_object_->setLow(start_object);
  end_object_->setHigh(end_object);

  current_object_->setDefaultValue(start_object);
  current_object_->setValue(start_object);
  current_object_->setLow(start_object);
  current_object_->setHigh(end_object);

  return;
}

double ParameterManager::getRegistrationMaxDistance(void) const
{
  return *registration_max_distance_;
}

void ParameterManager::addObjectParameters(ParameterDialog* parameter_dialog, bool with_objects)
{
  if (!with_objects)
    return;

  parameter_dialog->addParameter(start_object_);
  parameter_dialog->addParameter(end_object_);

  return;
}

bool ParameterManager::getObjectParameter(int& object)
{
  ParameterDialog parameter_dialog("object Parameter", MainWindow::getInstance());
  parameter_dialog.addParameter(current_object_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  object = *current_object_;
  return true;
}

void ParameterManager::getObjectparametersImpl(int& start_object, int& end_object, bool with_objects)
{
  if (!with_objects)
    return;

  if ((int)(*start_object_) > (int)(*end_object_))
  {
    int temp = *start_object_;
    start_object_->setValue(*end_object_);
    end_object_->setValue(temp);
  }

  start_object = *start_object_;
  end_object = *end_object_;

  return;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance)
{
  int place_holder_1, place_holder_2;
  return getRegistrationLUMParameters(segment_threshold, max_iterations, max_distance, place_holder_1, place_holder_2, false);

  return true;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance,
                                                 int& start_object, int& end_object, bool with_objects)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  addObjectParameters(&parameter_dialog, with_objects);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  getObjectparametersImpl(start_object, end_object, with_objects);

  return true;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance, int& object)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  parameter_dialog.addParameter(current_object_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  object = *current_object_;

  return true;
}

bool ParameterManager::getRegistrationICPParameters(int& max_iterations, double& max_distance, int& object, int& repeat_times)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  parameter_dialog.addParameter(current_object_);
  parameter_dialog.addParameter(repeat_times_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  object = *current_object_;
  repeat_times = *repeat_times_;

  return true;
}

double ParameterManager::getTriangleLength(void) const
{
  return *triangle_length_;
}

bool ParameterManager::getRegistrationParameters(int& object, int& segment_threshold)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(current_object_);
  parameter_dialog.addParameter(segment_threshold_);

  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  segment_threshold = *segment_threshold_;
  object = *current_object_;

  return true;
}


bool ParameterManager::getAutomaticRegistrationParameters(int& object, int& segment_threshold, int& max_iterations, double& max_distance)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(current_object_);
  parameter_dialog.addParameter(segment_threshold_);
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);

  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  segment_threshold = *segment_threshold_;
  object = *current_object_;
  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;

  return true;
}
