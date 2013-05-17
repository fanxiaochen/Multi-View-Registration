#pragma once
#ifndef PARAMETER_IMPL_H_
#define PARAMETER_IMPL_H_

#include <QVariant>
#include <QComboBox>

#include "parameter.h"


//////////////////////////////////////////////////////////////////////////////////////////////
template <class T> std::string
  EnumParameter<T>::valueTip()
{
  std::string tip("possible values: {");
  typename std::map<T, std::string>::const_iterator it = candidates_.begin();
  do
  {
    tip += it->second;
    ++ it;
    if (it != candidates_.end())
      tip += ", ";
  } while(it != candidates_.end());
  tip += "}";

  return (tip);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class T> QWidget *
  EnumParameter<T>::createEditor(QWidget *parent)
{
  QComboBox* editor = new QComboBox(parent);
  for (typename std::map<T, std::string>::const_iterator it = candidates_.begin();
    it != candidates_.end();
    ++ it)
  {
    editor->addItem(it->second.c_str());
  }

  return (editor);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class T> void
  EnumParameter<T>::setEditorData(QWidget *editor)
{
  QComboBox *comboBox = static_cast<QComboBox*>(editor);

  T value = T (*this);
  for (int i = 0, i_end = comboBox->count(); i < i_end; ++ i)
  {
    if (comboBox->itemText(i).toStdString() == value)
    {
      comboBox->setCurrentIndex(i);
      break;
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class T> void
  EnumParameter<T>::getEditorData(QWidget *editor)
{
  QComboBox *comboBox = static_cast<QComboBox*>(editor);
  std::map<T, std::string>::const_iterator it = candidates_.find(comboBox->itemText(comboBox->currentIndex()).toStdString());
  current_value_ = it->first;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class T> std::pair<QVariant, int>
EnumParameter<T>::toModelData()
{
  std::pair<QVariant, int> model_data;
  for (typename std::map<T, std::string>::const_iterator it = candidates_.begin();
    it != candidates_.end();
    ++ it) 
  {
    if (it->first == T(*this))
    {
      model_data.first = QString(it->second.c_str());
      break;
    }
  }
  model_data.second = Qt::EditRole;

  return (model_data);
}


#endif // PARAMETER_IMPL_H_
