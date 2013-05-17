#pragma once
#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <map>
#include <string>

#include <boost/any.hpp>

#include <QColor>
#include <QModelIndex>

class QWidget;
class QAbstractItemModel;


class Parameter
{
public:
  Parameter(const std::string& name, const std::string& description, const boost::any& value)
    : name_(name), description_(description), default_value_(value), current_value_(value){}
  virtual ~Parameter(void) {}

  const std::string& getName() const {return name_;}
  const std::string& getDescription()const {return description_;}
  void setDefaultValue(const boost::any& value) { default_value_ = value; }
  void setValue(const boost::any& value) { current_value_ = value; }
  void reset() {current_value_ = default_value_;}

  virtual std::string valueTip() = 0;
  virtual QWidget* createEditor(QWidget *parent) = 0;
  virtual void setEditorData(QWidget *editor) = 0;
  virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index);
  virtual std::pair<QVariant, int> toModelData() = 0;

protected:
  virtual void getEditorData(QWidget *editor) = 0;

  std::string     name_;
  std::string     description_;
  boost::any      default_value_;
  boost::any      current_value_;
};

class BoolParameter : public Parameter
{
public:
  BoolParameter(const std::string& name, const std::string& description, bool value)
    : Parameter(name, description, value){}
  virtual ~BoolParameter(){}
  operator bool() const {return boost::any_cast<bool>(current_value_);}
  virtual std::string valueTip();
  virtual QWidget* createEditor(QWidget *parent);
  virtual void setEditorData(QWidget *editor);
  virtual std::pair<QVariant, int> toModelData();

protected:
  virtual void getEditorData(QWidget *editor);
};

class IntParameter : public Parameter
{
public:
  IntParameter(const std::string& name, const std::string& description, int value, int low, int high, int step=1)
    : Parameter(name, description, value), low_(low), high_(high), step_(step){}
  virtual ~IntParameter(){}
  operator int() const {return boost::any_cast<int>(current_value_);}
  virtual std::string valueTip();
  virtual QWidget* createEditor(QWidget *parent);
  virtual void setEditorData(QWidget *editor);
  virtual std::pair<QVariant, int> toModelData();
  int getDefaultValue(void) const {return boost::any_cast<int>(default_value_);}
  void setLow(int low) { low_ = low; }
  int getLow(void) const { return low_; }
  void setHigh(int high) { high_ = high; }
  int getHigh(void) const { return high_; }
  void setStep(int step) { step_ = step; }

protected:
  virtual void getEditorData(QWidget *editor);

  int     low_;
  int     high_;
  int     step_;
};

template <class T>
class EnumParameter : public Parameter
{
public:
  EnumParameter(const std::string& name, const std::string& description, T value, const std::map<T, std::string>& candidates)
    : Parameter(name, description, value), candidates_(candidates){}
  virtual ~EnumParameter(){}
  operator T() const {return boost::any_cast<T>(current_value_);}
  virtual std::string valueTip();
  virtual QWidget* createEditor(QWidget *parent);
  virtual void setEditorData(QWidget *editor);
  virtual std::pair<QVariant, int> toModelData();

protected:
  virtual void getEditorData(QWidget *editor);
  const std::map<T, std::string> candidates_;
};

class DoubleParameter : public Parameter
{
public:
  DoubleParameter(const std::string& name, const std::string& description, double value, double low, double high, double step=0.01)
    : Parameter(name, description, value), low_(low), high_(high), step_(step){}
  virtual ~DoubleParameter(){}
  operator double() const {return boost::any_cast<double>(current_value_);}
  virtual std::string valueTip();
  virtual QWidget* createEditor(QWidget *parent);
  virtual void setEditorData(QWidget *editor);
  virtual std::pair<QVariant, int> toModelData();
  double getDefaultValue(void) const {return boost::any_cast<double>(default_value_);}
  void setLow(double low) { low_ = low; }
  double getLow(void) const {return low_;}
  void setHigh(double high) { high_ = high; }
  double getHigh(void) const {return high_;}
  void setStep(double step) { step_ = step; }

protected:
  virtual void getEditorData(QWidget *editor);

  double  low_;
  double  high_;
  double  step_;
};

class ColorParameter : public Parameter
{
public:
  ColorParameter(const std::string& name, const std::string& description, QColor value)
    : Parameter(name, description, value){}
  virtual ~ColorParameter(){}
  operator QColor() const {return boost::any_cast<QColor>(current_value_);}
  virtual std::string valueTip();
  virtual QWidget* createEditor(QWidget *parent);
  virtual void setEditorData(QWidget *editor);
  virtual std::pair<QVariant, int> toModelData();

protected:
  virtual void getEditorData(QWidget *editor);
};

#include "impl/parameter.hpp"

#endif // PARAMETER_H_
