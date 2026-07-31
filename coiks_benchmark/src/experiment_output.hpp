#ifndef COIKS_BENCHMARK_EXPERIMENT_OUTPUT_HPP
#define COIKS_BENCHMARK_EXPERIMENT_OUTPUT_HPP

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

inline bool isExperimentDirName(const std::string &name)
{
  if (name.size() != 3)
    return false;
  return std::isdigit(static_cast<unsigned char>(name[0])) &&
         std::isdigit(static_cast<unsigned char>(name[1])) &&
         std::isdigit(static_cast<unsigned char>(name[2]));
}

// 在 output_root 下创建下一个实验目录：001, 002, 003, ...
inline std::string createNextExperimentDir(const std::string &output_root)
{
  mkdir(output_root.c_str(), 0755);

  int max_id = 0;
  DIR *dir = opendir(output_root.c_str());
  if (dir)
  {
    while (dirent *ent = readdir(dir))
    {
      const std::string name = ent->d_name;
      if (isExperimentDirName(name))
        max_id = std::max(max_id, std::stoi(name));
    }
    closedir(dir);
  }

  const int next_id = max_id + 1;
  std::ostringstream oss;
  oss << output_root << "/" << std::setw(3) << std::setfill('0') << next_id;
  const std::string exp_dir = oss.str();

  if (mkdir(exp_dir.c_str(), 0755) != 0)
  {
    std::cerr << "[ERROR] 无法创建实验结果目录: " << exp_dir << std::endl;
  }
  return exp_dir;
}

inline std::string makeResultCsvPath(const std::string &exp_dir,
                                     const std::string &solver_name,
                                     int robot_id,
                                     const std::string &name_suffix = "")
{
  return exp_dir + "/" + solver_name + name_suffix + "_r" +
         std::to_string(robot_id) + ".csv";
}

#endif
