#ifndef TESSERACT_EXAMPLES_UR5_TRAJOPT_H
#define TESSERACT_EXAMPLES_UR5_TRAJOPT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/example.h>

namespace tesseract_examples
{

class UR5Trajopt : public Example
{
public:
  UR5Trajopt(tesseract_environment::Environment::Ptr env,
                      tesseract_visualization::Visualization::Ptr plotter = nullptr,
                      bool ifopt = false,
                      bool debug = false);
  ~UR5Trajopt() override = default;
  UR5Trajopt(const UR5Trajopt&) = default;
  UR5Trajopt& operator=(const UR5Trajopt&) = default;
  UR5Trajopt(UR5Trajopt&&) = default;
  UR5Trajopt& operator=(UR5Trajopt&&) = default;

  bool run() override final;

private:
  bool ifopt_;
  bool debug_;
  static tesseract_environment::Command::Ptr addSphere();
};

}

#endif