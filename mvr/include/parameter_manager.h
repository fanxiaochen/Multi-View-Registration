#pragma once
#ifndef PARAMETER_MANAGER_H_
#define PARAMETER_MANAGER_H_

#include <QString>

class IntParameter;
class DoubleParameter;
class BoolParameter;
class ParameterDialog;

class ParameterManager
{
public:
  static ParameterManager& getInstance() {
    static ParameterManager theSingleton;
    return theSingleton;
  }

  void initObjectNumbers(void);


  double getRegistrationMaxDistance(void) const;


  bool getObjectParameter(int& object);
  bool getObjectParameters(int& start_object, int& end_object, int& downsampling);


  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance);
  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance,
    int& start_object, int& end_object, bool with_objects=true);
  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance, int& object);
  bool getRegistrationICPParameters(int& max_iterations, double& max_distance, int& object, int& repeat_times);

protected:
  void addObjectParameters(ParameterDialog* parameter_dialog, bool with_objects);
  void getObjectparametersImpl(int& start_object, int& end_object, bool with_objects);
private:
  ParameterManager(void);
  ParameterManager(const ParameterManager &) {}            // copy ctor hidden
  ParameterManager& operator=(const ParameterManager &) {return (*this);}   // assign op. hidden
  virtual ~ParameterManager();

  IntParameter*                                       registration_max_iterations_;
  DoubleParameter*                                    registration_max_distance_;

  IntParameter*                                       start_object_;
  IntParameter*                                       end_object_;
  IntParameter*                                       current_object_;

  IntParameter*                                       repeat_times_;

};

#endif // PARAMETER_MANAGER_H_
