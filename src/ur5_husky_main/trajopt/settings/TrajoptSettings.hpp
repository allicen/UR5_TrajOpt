#pragma once
#ifndef TRAJOPTSETTINGS_HPP
#define TRAJOPTSETTINGS_HPP

#include <tesseract_common/macros.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <iostream>
#include <vector>
#include <string>
#include <json/json.h>

using namespace trajopt;

class TrajoptSettings
{
public:
  TrajoptSettings(std::string executor_name_, 
                  std::string profile_name_,
                  std::string description_,
                  std::vector<DefaultCompositeProfileSettings> default_composite_profile_,
                  std::vector<DefaultPlanProfileSettings> default_plan_profile_,
                  std::vector<DefaultSolverProfileSettings> DefaultSolverProfile_) {

    executor_name = executor_name_;
    profile_name = profile_name_;
    description = description_;
    default_composite_profile = default_composite_profile_;
    default_plan_profile = default_plan_profile_;
    default_solver_profile = default_solver_profile_;
  };
  ~TrajoptSettings() = default;


private:
      // Название исполнителя
    std::string executor_name;

    //Название профиля
    std::string profile_name;

    // Произвольное описание
    std::string description;

    // Профили планировщика
    std::vector<DefaultCompositeProfileSettings> default_composite_profile;
    std::vector<DefaultPlanProfileSettings> default_plan_profile;
    std::vector<DefaultSolverProfileSettings> default_solver_profile;

    std::string task_name;

    // Программа, которая подается в планировщик для выполнения
    Json::Value program;
  
};

#endif