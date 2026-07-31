
#include <coiks/coiks.hpp>
#include <coiks/ik_solver.hpp>

#include <csignal>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

// Orocos KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>

// Eigen conversions
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

// Trac_IK
#include <trac_ik/trac_ik.hpp>

// KDL Parser
#include <kdl_parser/kdl_parser.hpp>

#include <coiks/solution_set.hpp>

#include "experiment_output.hpp"

using namespace coiks;
namespace pin = pinocchio;

class BenchmarkKDL : public IkSolver
{
  public:
  BenchmarkKDL(const std::string &_urdf_file, const std::string &_base_link,
               const std::string &_ee_link, const std::string &_ik_solver,
               double _max_time, double _max_error, int _max_iter, double _dt)
      : initialized_(false), max_error_(_max_error), max_iter_(_max_iter),
        urdf_file_(_urdf_file), base_link_(_base_link), ee_link_(_ee_link),
        dt_(_dt), solver_name_("kdl")
  {
    initialize_kdl();
  }

  ~BenchmarkKDL() {}

  bool initialize_kdl()
  {
    std::cout << "正在初始化 KDL，最大误差: " << max_error_
              << " 最大迭代次数:" << max_iter_ << " 时间步长:" << dt_ << std::endl;

    double maxtime = 0.005;

    // Parsing URDF
    if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree_))
    {
      std::cerr << "[ERROR] " << "构建 KDL 树失败" << std::endl;
      return false;
    }
    bool exit_value = kdl_tree_.getChain(base_link_, ee_link_, kdl_chain_);
    // Resize variables
    n_joints_ = kdl_chain_.getNrOfJoints();
    qtmp_.resize(kdl_chain_.getNrOfJoints());
    nominal_.resize(kdl_chain_.getNrOfJoints());
    ll_.resize(kdl_chain_.getNrOfJoints());
    ul_.resize(kdl_chain_.getNrOfJoints());

    //   Load the urdf model
    pin::Model model_;
    pin::urdf::buildModel(urdf_file_, model_);

    // Getting Joints Limits
    q_ul_ = model_.upperPositionLimit;
    q_ll_ = model_.lowerPositionLimit;

    // Storing Joint limits
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      ll_.data(i) = q_ll_[i];
      ul_.data(i) = q_ul_[i];
    }

    assert(kdl_chain_.getNrOfJoints() == ll_.data.size());
    assert(kdl_chain_.getNrOfJoints() == ul_.data.size());

    kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    kdl_vik_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    kdl_ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(
        kdl_chain_, ll_, ul_, *kdl_fk_solver_, *kdl_vik_solver_, max_iter_,
        max_error_));

    // Initialize nominal vector
    for (uint j = 0; j < nominal_.data.size(); j++)
    {
      nominal_(j) = (ll_(j) + ul_(j)) / 2.0;
    }
    std::cout << "KDL 初始化完成" << std::endl;
    initialized_ = true;

    return true;
  }

  // * KDL solver
  int IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d, VectorXd &q_out)
  {
    bool          success = false;
    int           rc;
    KDL::JntArray qd(n_joints_);
    KDL::Frame    ee;

    Affine3d Tdes;
    Tdes.linear() = x_Fee_d.rotation();
    Tdes.translation() = x_Fee_d.translation();

    tf::transformEigenToKDL(Tdes, ee);

    rc = kdl_ik_solver_->CartToJnt(nominal_, ee, qd);

    if (rc >= 0)
    {
      // std::cout << "Solution found" << std::endl;
      q_out = VectorXd::Map(&qd.data[0], qd.data.size());
      // std::cout << "Solution found: " << qd.data * (180 / M_PI) << std::endl;
      // std::cout << "Time IK [us]: " << duration_cb.count() << " usec";

      success = true;
      return 0;
      // std::cerr << "[WARN] " << "IK solution found: " <<
      // des_joint_pos_.transpose() << std::endl
    }
    else
    {
      // std::cerr << "[WARN] " << "No IK solution found" << std::endl
      success = false;
      return 1;
    }
    // return success;
  }

  int         get_n_joints() { return n_joints_; }
  std::string get_solver_name() { return solver_name_; }

  private:
  // Temporary variables for KDL
  KDL::Tree  kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray nominal_;
  KDL::JntArray qtmp_;
  KDL::JntArray ll_, ul_;
  KDL::Frame    xtmp_;
  KDL::Jacobian Jtmp_;

  KDL::Twist    xdot_temp_;
  KDL::JntArray qdot_tmp_;

  // KDL
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv>      kdl_vik_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>     kdl_ik_solver_;

  std::string urdf_file_;

  std::string base_link_;
  std::string ee_link_;
  VectorXd    q_ul_;
  VectorXd    q_ll_;

  int         n_joints_;
  std::string solver_name_;
  int         max_iter_;
  double      max_error_;

  bool   initialized_;
  double dt_;
};

class BenchmarkTRACIK : public IkSolver
{
  public:
  BenchmarkTRACIK(const std::string &_urdf_file, const std::string &_base_link,
                  const std::string &_ee_link, const std::string &_ik_solver,
                  const std::string &_solver_mode, double _max_time,
                  double _max_error, int _max_iter, double _dt)
      : initialized_(false), max_error_(_max_error), max_time_(_max_time),
        urdf_file_(_urdf_file), base_link_(_base_link), ee_link_(_ee_link),
        dt_(_dt), solver_name_("trac_ik"), solver_mode_(_solver_mode)
  {
    initialize_tracik();
  }

  ~BenchmarkTRACIK() {}

  bool initialize_tracik()
  {
    std::cout << "正在初始化 TRACIK，最大误差: " << max_error_
              << " 最大时间:" << max_time_ << " 时间步长:" << dt_ << std::endl;

    // Parsing URDF
    if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree_))
    {
      std::cerr << "[ERROR] " << "构建 KDL 树失败" << std::endl;
      return false;
    }
    bool exit_value = kdl_tree_.getChain(base_link_, ee_link_, kdl_chain_);

    // Resize variables
    n_joints_ = kdl_chain_.getNrOfJoints();
    qtmp_.resize(kdl_chain_.getNrOfJoints());
    nominal_.resize(kdl_chain_.getNrOfJoints());
    ll_.resize(kdl_chain_.getNrOfJoints());
    ul_.resize(kdl_chain_.getNrOfJoints());

    //   Load the urdf model
    pin::Model model_;
    pin::urdf::buildModel(urdf_file_, model_);

    // Getting Joints Limits
    q_ul_ = model_.upperPositionLimit;
    q_ll_ = model_.lowerPositionLimit;

    // Storing Joint limits
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      ll_.data(i) = q_ll_[i];
      ul_.data(i) = q_ul_[i];
    }

    assert(kdl_chain_.getNrOfJoints() == ll_.data.size());
    assert(kdl_chain_.getNrOfJoints() == ul_.data.size());

    if (solver_mode_ == "speed")
      tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, ll_, ul_, max_time_,
                                                max_error_, TRAC_IK::Speed));
    else if (solver_mode_ == "distance")
      tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, ll_, ul_, max_time_,
                                                max_error_, TRAC_IK::Distance));
    else if (solver_mode_ == "manipulability")
      tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, ll_, ul_, max_time_,
                                                max_error_, TRAC_IK::Manip1));
    else
    {
      std::cerr << "[WARN] " << "求解器模式无效，将使用速度模式" << std::endl;
      tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, ll_, ul_, max_time_,
                                                max_error_, TRAC_IK::Speed));
    }

    // Initialize nominal vector
    for (uint j = 0; j < nominal_.data.size(); j++)
    {
      nominal_(j) = (ll_(j) + ul_(j)) / 2.0;
    }
    std::cout << "TRACIK 初始化完成" << std::endl;

    return true;
  }

  // * TRAC-IK solver
  int IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d, VectorXd &q_out)
  {
    bool          success = false;
    int           rc;
    KDL::JntArray qd(n_joints_);
    KDL::Frame    ee;

    Affine3d Tdes;
    Tdes.linear() = x_Fee_d.rotation();
    Tdes.translation() = x_Fee_d.translation();

    tf::transformEigenToKDL(Tdes, ee);

    rc = tracik_solver_->CartToJnt(nominal_, ee, qd);

    if (rc >= 0)
    {
      q_out = VectorXd::Map(&qd.data[0], qd.data.size());
      success = true;
      return 0;
    }
    else
    {
      success = false;
      return 1;
    }
  }

  int         get_n_joints() { return n_joints_; }
  std::string get_solver_name() { return solver_name_; }

  private:
  std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

  std::string urdf_file_;
  std::string solver_mode_;

  std::string base_link_;
  std::string ee_link_;
  VectorXd    q_ul_;
  VectorXd    q_ll_;

  int         n_joints_;
  std::string solver_name_;
  double      max_error_;
  double      max_time_;

  // Temporary variables for KDL
  KDL::Tree  kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray nominal_;
  KDL::JntArray qtmp_;
  KDL::JntArray ll_, ul_;

  bool   initialized_;
  double dt_;
};

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void solveIkFromRandomList(std::string urdf_file_, std::string qinit_file_,
                           std::string target_file_, int num_samples,
                           IkSolver *ik_solver, std::string ee_link_,
                           bool print_all, int robot_id,
                           const std::string &output_dir)
{
  double                  total_time = 0;
  double                  total_time_it = 0;
  double                  success_time_it = 0;
  uint                    n_success = 0;
  IKSolutionSet::Solution bestSolution;

  //   Pinocchio variables
  pin::Model      model_;
  pin::Data       mdl_data_;
  pin::FrameIndex ee_id_;

  VectorXd q_ul_;
  VectorXd q_ll_;
  VectorXd q_sol_;

  // std::vector<VectorXd> q_solutions_;
  std::vector<double> errors_;
  int                 n_joints_;

  std::fstream fout;
  int          prob_id = 0;

  //   Load the urdf model
  pin::urdf::buildModel(urdf_file_, model_);
  ee_id_ = model_.getFrameId(ee_link_);

  n_joints_ = model_.nq;
  q_sol_.resize(n_joints_);
  // q_solutions_.clear();
  errors_.clear();

  // Getting Joints Limits
  q_ul_ = model_.upperPositionLimit;
  q_ll_ = model_.lowerPositionLimit;

  std::cout << "使用求解器求解 IK: " << ik_solver->get_solver_name() << "，处理 "
            << num_samples << " 个随机配置，目标链接为 " << ee_link_
            << std::endl;

  // Create desired number of valid, random joint configurations
  std::vector<VectorXd> qinit_list;
  std::vector<pin::SE3> xdes_list;
  VectorXd              qinit(ik_solver->get_n_joints());

  // Read qinit values from CSV file
  std::cout << "正在读取文件: " << qinit_file_ << std::endl;
  std::string   line;
  std::ifstream file(qinit_file_);

  if (file.is_open())
  {
    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line))
    {
      std::stringstream   linestream(line);
      std::string         value;
      std::vector<double> values;

      // Read problem_id
      std::getline(linestream, value, ',');
      int problem_id = std::stoi(value);

      // Read joint values
      for (int i = 0; i < qinit.size(); ++i)
      {
        std::getline(linestream, value, ',');
        qinit(i) = std::stod(value);
      }

      qinit_list.push_back(qinit);
    }
    file.close();
  }
  else
  {
    std::cerr << "无法打开文件: " << qinit_file_ << std::endl;
  }

  // Read target values from CSV file
  std::cout << "正在读取文件: " << target_file_ << std::endl;
  std::ifstream file2(target_file_);

  if (file2.is_open())
  {
    // Skip the header line
    std::getline(file2, line);

    while (std::getline(file2, line))
    {
      std::stringstream   linestream(line);
      std::string         value;
      std::vector<double> values;

      // Read problem_id
      std::getline(linestream, value, ',');
      int problem_id = std::stoi(value);

      // Read target values
      pin::SE3 xdes;
      for (int i = 0; i < 3; ++i)
      {
        std::getline(linestream, value, ',');
        xdes.translation()(i) = std::stod(value);
      }
      Eigen::Quaterniond quat;
      std::getline(linestream, value, ',');
      quat.w() = std::stod(value);

      std::getline(linestream, value, ',');
      quat.x() = std::stod(value);

      std::getline(linestream, value, ',');
      quat.y() = std::stod(value);

      std::getline(linestream, value, ',');
      quat.z() = std::stod(value);

      xdes.rotation() = quat.toRotationMatrix();
      xdes_list.push_back(xdes);
    }
    file2.close();
  }
  else
  {
    std::cerr << "无法打开文件: " << target_file_ << std::endl;
  }

  pin::Data mdl_data(model_);

  const std::string csv_path =
      makeResultCsvPath(output_dir, ik_solver->get_solver_name(), robot_id);
  fout.open(csv_path, std::ios::out);
  if (!fout.is_open())
  {
    std::cerr << "[ERROR] 无法打开结果文件: " << csv_path << std::endl;
  }
  else
  {
    std::cout << "[INFO] 结果写入: " << csv_path << std::endl;
  }

  if (ik_solver->get_solver_name() == "kdl" ||
      ik_solver->get_solver_name() == "trac_ik")
  {
    fout << "idx,time,q_init,q_sol\n";
  }
  else
  {
    fout << "idx,res,time,solver_id,sol_id,error,q_init,q_sol\n";
  }

  prob_id = 0;

  auto start_cb_time = std::chrono::high_resolution_clock::now();
  auto start_it_time = std::chrono::high_resolution_clock::now();
  auto stop_it_time = std::chrono::high_resolution_clock::now();

  for (uint i = 0; i < num_samples; i++)
  {
    // Initialize initial configuration
    VectorXd q_init = pin::neutral(model_);
    q_init = qinit_list[i];

    // Initializing solution
    VectorXd q_sol = pin::neutral(model_);

    // Selecting target pose
    pin::SE3 x_des = xdes_list[i];

    prob_id++;
    if (print_all)
      std::cout << "\n[问题 " << i
                << "] 正在求解关节配置: " << q_init.transpose() << std::endl;

    if (print_all)
      std::cout << "[问题 " << i
                << "] 正在求解位置: " << x_des.translation().transpose()
                << std::endl;
    start_it_time = std::chrono::high_resolution_clock::now();

    // Call IK Solver
    int res = 1;
    if (ik_solver->get_solver_name() == "kdl" ||
        ik_solver->get_solver_name() == "trac_ik")
    {
      res = ik_solver->IkSolve(q_init, x_des, q_sol);
    }
    else
    {
      res = ik_solver->IkSolve(q_init, x_des, q_sol, bestSolution);
    }

    stop_it_time = std::chrono::high_resolution_clock::now();
    auto duration_it = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_it_time - start_it_time);
    total_time_it += duration_it.count();

    if (res == 0 && ik_solver->get_solver_name() != "kdl" &&
        ik_solver->get_solver_name() != "trac_ik" && print_all)
    {
      std::cout << "找到 IK 解: " << q_sol.transpose() << std::endl;

      // You can now access the best solution details
      std::cout << "最佳解指标: " << bestSolution.metrics["error"]
                << std::endl;
      std::cout << "最佳解关节位置: "
                << bestSolution.configuration.transpose() << std::endl;
      // Access other fields of bestSolution as needed
    }

    if (res && print_all)
      std::cerr << "[WARN] " << "未找到解" << std::endl;

    if (print_all)
    {
      std::cout << "耗时: " << duration_it.count() << " [微秒]" << std::endl;
      if (!res)
      {
        std::cout << "解: " << q_sol.transpose() << std::endl;
      }
    }

    if (ik_solver->get_solver_name() != "kdl" &&
        ik_solver->get_solver_name() != "trac_ik")
    {
      if (!res)
      {
        fout << prob_id << "," << res << "," << duration_it.count() << ","
             << bestSolution.solver_id << "," << bestSolution.solution_id << ","
             << bestSolution.metrics["error"] << "," << q_init.transpose()
             << "," << bestSolution.configuration.transpose() << "\n";
      }
      else
        fout << prob_id << "," << res << "," << duration_it.count() << "," << 0
             << "," << 0 << "," << 0 << "," << q_init.transpose() << ","
             << q_init.transpose() << "\n";
    }

    if (!res)
    {
      success_time_it += duration_it.count();
      n_success++;

      if (ik_solver->get_solver_name() == "kdl")
      {
        fout << prob_id << "," << duration_it.count() << ","
             << q_init.transpose() << "," << q_sol.transpose() << "\n";
      }
      else if (ik_solver->get_solver_name() == "trac_ik")
      {
        fout << prob_id << "," << duration_it.count() << ","
             << q_init.transpose() << "," << q_sol.transpose() << "\n";
      }
    }
  }

  auto stop_cb_time = std::chrono::high_resolution_clock::now();
  auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(
      stop_cb_time - start_cb_time);

  total_time = duration_cb.count();

  std::cout << "------------------------------" << std::endl;
  std::cout << "汇总:" << std::endl;

  std::cout << ik_solver->get_solver_name() << " 找到 " << n_success
            << " 个解（" << 100.0 * n_success / num_samples
            << "\%），总平均耗时 " << total_time / num_samples
            << " 微秒/配置。求解平均耗时 "
            << total_time_it / num_samples << " 微秒/配置。"
            << "成功求解平均耗时 " << success_time_it / n_success
            << " 微秒/配置。" << std::endl;
  if (ik_solver->get_solver_name() == "codcs")
  {
    ik_solver->printStatistics();
  }
  std::cout << "------------------------------" << std::endl;

  fout.close();

  return;
}

bool kill_process = false;
void SigIntHandler(int signal)
{
  kill_process = true;
  std::cout << "[INFO] " << "收到关闭信号" << std::endl;
}

int main(int argc, char **argv)
{
  std::cout << "[INFO] " << "COIKS、TRAC-IK 与 KDL 无约束 IK 基准测试" << std::endl;
  ros::init(argc, argv, "coiks_benchmark");
  ros::NodeHandle nh;
  std::signal(SIGINT, SigIntHandler);

  // ROS parameters
  std::string ik_solver;
  int         max_iter;
  int         n_random_config;
  double      max_time;
  double      max_error;
  double      dt;
  int         print_all;

  if (!nh.getParam("ik_solver", ik_solver))
  {
    ik_solver = "tracik";
  }

  std::string solve_mode;
  if (!nh.getParam("solve_mode", solve_mode))
  {
    solve_mode = "speed";
  }

  std::string ee_link_name;
  if (!nh.getParam("ee_link_name", ee_link_name))
  {
    ee_link_name = "ee_link";
  }
  if (!nh.getParam("max_iter", max_iter))
  {
    max_iter = 10000;
  }
  if (!nh.getParam("max_time", max_time))
  {
    max_time = 5e-3;
  }
  if (!nh.getParam("max_error", max_error))
  {
    max_error = 1e-5;
  }
  if (!nh.getParam("delta_integration", dt))
  {
    dt = 1.0;
  }
  std::string error_type;
  if (!nh.getParam("error_type", error_type))
  {
    error_type = "log6";
  }

  //* Esperiment variables
  int robot_id;
  if (!nh.getParam("robot_id", robot_id))
  {
    robot_id = 0;
  }
  if (!nh.getParam("n_random_config", n_random_config))
  {
    n_random_config = 100;
  }

  // Additional variables
  std::string step_size_method;
  if (!nh.getParam("step_size_method", step_size_method))
  {
    step_size_method = "fixed";
  }

  std::string seed_method;
  if (!nh.getParam("seed_method", seed_method))
  {
    seed_method = "current";
  }

  std::string limiting_method;
  if (!nh.getParam("limiting_method", limiting_method))
  {
    limiting_method = "random";
  }

  std::string pinv_method;
  if (!nh.getParam("pinv_method", pinv_method))
  {
    pinv_method = "cod";
  }

  //* INVJ Variables
  double invj_Ke1;
  if (!nh.getParam("invj_Ke1", invj_Ke1))
  {
    invj_Ke1 = 1.0;
  }

  int invj_multi_n_solvers;
  if (!nh.getParam("invj_multi_n_solvers", invj_multi_n_solvers))
  {
    invj_multi_n_solvers = 2;
  }

  int invj_max_stagnation_iter;
  if (!nh.getParam("invj_max_stagnation_iter", invj_max_stagnation_iter))
  {
    invj_max_stagnation_iter = 10;
  }

  double invj_improvement_threshold;
  if (!nh.getParam("invj_improvement_threshold", invj_improvement_threshold))
  {
    invj_improvement_threshold = 1e-5;
  }

  //* NLO Variables
  std::string nlo_linear_solver;
  if (!nh.getParam("nlo_linear_solver", nlo_linear_solver))
  {
    nlo_linear_solver = "ma57";
  }

  double mu0;
  if (!nh.getParam("nlo_mu0", mu0))
  {
    mu0 = 1.0;
  }

  double mu1;
  if (!nh.getParam("nlo_mu1", mu1))
  {
    mu1 = 0.005;
  }

  double mu2;
  if (!nh.getParam("nlo_mu2", mu2))
  {
    mu2 = 0.001;
  }

  double mu3;
  if (!nh.getParam("nlo_mu3", mu3))
  {
    mu3 = 100.0;
  }

  double mu4;
  if (!nh.getParam("nlo_mu4", mu4))
  {
    mu4 = 0.01;
  }

  bool nlo_concurrent;
  if (!nh.getParam("nlo_concurrent", nlo_concurrent))
  {
    nlo_concurrent = false;
  }
  std::string nlo_error_type;
  if (!nh.getParam("nlo_error_type", nlo_error_type))
  {
    nlo_error_type = "log3";
  }
  int nlo_concurrent_iterations;
  if (!nh.getParam("nlo_concurrent_iterations", nlo_concurrent_iterations))
  {
    nlo_concurrent_iterations = 5;
  }
  std::string nlo_warm_start;
  if (!nh.getParam("nlo_warm_start", nlo_warm_start))
  {
    nlo_warm_start = "yes";
  }

  //* QP variables
  double qp_K_t1;
  if (!nh.getParam("qp_K_t1", qp_K_t1))
  {
    qp_K_t1 = 1.0;
  }
  double qp_Kr_t1;
  if (!nh.getParam("qp_Kr_t1", qp_Kr_t1))
  {
    qp_Kr_t1 = 0.00001;
  }
  double qp_Kw_p1;
  if (!nh.getParam("qp_Kw_p1", qp_Kw_p1))
  {
    qp_Kw_p1 = 0.00001;
  }
  double qp_Kd_p1;
  if (!nh.getParam("qp_Kd_p1", qp_Kd_p1))
  {
    qp_Kd_p1 = 0.00001;
  }
  bool qp_warm_start;
  if (!nh.getParam("qp_warm_start", qp_warm_start))
  {
    qp_warm_start = true;
  }

  //* Printing variables
  std::string time_stats;
  if (!nh.getParam("solv_time_stats", time_stats))
  {
    time_stats = "no";
  }
  int verb_level;
  if (!nh.getParam("solv_verb_level", verb_level))
  {
    verb_level = 0;
  }
  if (!nh.getParam("print_all", print_all))
  {
    print_all = 0;
  }

  bool tracik_enable;
  if (!nh.getParam("tracik_enable", tracik_enable))
  {
    tracik_enable = true;
  }

  bool kdl_enable;
  if (!nh.getParam("kdl_enable", kdl_enable))
  {
    kdl_enable = true;
  }

  // Setting up URDF path
  std::string pkg_path = ros::package::getPath("coiks_benchmark");

  std::string urdf_path;
  std::string qinit_path;
  std::string target_path;

  switch (robot_id)
  {
  case 0:
    urdf_path = pkg_path + std::string("/urdf/") + "toyota/hsr/hsr_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_hsr_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_hsr_random_10000samples.csv";
    break;

  case 1:
    urdf_path = pkg_path + std::string("/urdf/") + "denso/vs050/vs050.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_vs050_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_vs050_random_10000samples.csv";
    break;

  case 2:
    urdf_path = pkg_path + std::string("/urdf/") + "ufactory/xarm6/xarm6.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_xarm6_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_xarm6_random_10000samples.csv";
    break;

  case 3:
    urdf_path = pkg_path + std::string("/urdf/") + "abb/irb140/irb140.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_irb140_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_irb140_random_10000samples.csv";
    break;

  case 4:
    urdf_path = pkg_path + std::string("/urdf/") + "kinova/jaco/jaco_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_jaco_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_jaco_random_10000samples.csv";
    break;

  case 5:
    urdf_path = pkg_path + std::string("/urdf/") + "ur/ur5/ur5.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_ur5_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_ur5_random_10000samples.csv";
    break;

  case 6:
    urdf_path = pkg_path + std::string("/urdf/") + "ur/ur10/ur10.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_ur10_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_ur10_random_10000samples.csv";
    break;

  case 7:
    urdf_path = pkg_path + std::string("/urdf/") + "kinova/gen3/gen3.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_gen3_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_gen3_random_10000samples.csv";
    break;

  case 8:
    urdf_path = pkg_path + std::string("/urdf/") + "kuka/iiwa/iiwa.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_iiwa_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_iiwa_random_10000samples.csv";
    break;

  case 9:
    urdf_path = pkg_path + std::string("/urdf/") + "ufactory/xarm7/xarm7.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_xarm7_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_xarm7_random_10000samples.csv";
    break;

  case 10:
    urdf_path = pkg_path + std::string("/urdf/") + "franka/panda.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_panda_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_panda_random_10000samples.csv";
    break;

  case 11:
    urdf_path = pkg_path + std::string("/urdf/") + "rethink/sawyer/sawyer.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_sawyer_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_sawyer_random_10000samples.csv";
    break;

  case 12:
    urdf_path = pkg_path + std::string("/urdf/") + "abb/yumi/yumi.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_yumi_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_yumi_random_10000samples.csv";
    break;

  case 13:
    urdf_path = pkg_path + std::string("/urdf/") + "fetch/fetch_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_fetch_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_fetch_random_10000samples.csv";
    break;

  case 14:
    urdf_path = pkg_path + std::string("/urdf/") + "tiago/tiago.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_tiago_random_10000samples.csv";
    target_path = pkg_path + std::string("/data/") +
                  "targets_robot_tiago_random_10000samples.csv";
    break;

  default:
    std::cerr << "[ERROR] " << "无法识别运动学树 ID。" << std::endl;
    return -1;
  }
  std::cout << "[INFO] " << "正在使用此处找到的 URDF: " << urdf_path << std::endl;

  // Creating solver options class
  SolverOptions so;

  so.solve_mode_ = solve_mode;
  so.error_type_ = error_type;
  so.step_size_method_ = step_size_method;
  so.seed_method_ = seed_method;
  so.limiting_method_ = limiting_method;
  so.pinv_method_ = pinv_method;

  so.invj_Ke1_ = invj_Ke1;

  so.invj_multi_n_solvers_ = invj_multi_n_solvers;
  so.invj_max_stagnation_iterations_ = invj_max_stagnation_iter;
  so.invj_improvement_threshold_ = invj_improvement_threshold;

  so.nlo_linear_solver_ = nlo_linear_solver;
  so.cost_coeff_.push_back(mu0);
  so.cost_coeff_.push_back(mu1);
  so.cost_coeff_.push_back(mu2);
  so.cost_coeff_.push_back(mu3);
  so.cost_coeff_.push_back(mu4);
  so.nlo_concurrent_ = nlo_concurrent;
  so.nlo_error_type_ = nlo_error_type;
  so.nlo_concurrent_iterations_ = nlo_concurrent_iterations;
  so.nlo_warm_start_ = nlo_warm_start;

  so.qp_K_t1_ = qp_K_t1;
  so.qp_Kr_t1_ = qp_Kr_t1;
  so.qp_Kw_p1_ = qp_Kw_p1;
  so.qp_warm_start_ = qp_warm_start;

  so.time_stats_ = time_stats;
  so.verb_level_ = verb_level;

  std::cout << "[INFO] " << "误差类型: " << so.error_type_ << std::endl;

  // Initialiing KDL
  std::cout << "[INFO] " << "正在启动 KDL" << std::endl;
  BenchmarkKDL kdl_ik(urdf_path, "base_link", "link_ee", ik_solver, max_time,
                      max_error, max_iter, dt);

  // Initialiing TRAC-IK
  std::cout << "[INFO] " << "正在启动 TRAC-IK" << std::endl;
  BenchmarkTRACIK trac_ik(urdf_path, "base_link", "link_ee", ik_solver,
                          solve_mode, max_time, max_error, max_iter, dt);

  // Initialiing COIKS
  std::cout << "[INFO] " << "正在启动 CODCS-IK" << std::endl;
  COIKS coiks(urdf_path, "base_link", "link_ee", ik_solver, so, max_time,
              max_error, max_iter, dt);

  const std::string exp_dir =
      createNextExperimentDir(pkg_path + "/output");
  std::cout << "[INFO] 实验结果目录: " << exp_dir << std::endl;

  if (kdl_enable)
  {
    std::cerr << "[WARN] " << "正在对 KDL 运行随机配置测试" << std::endl;
    solveIkFromRandomList(urdf_path, qinit_path, target_path, n_random_config,
                          &kdl_ik, "link_ee", (print_all & 1), robot_id,
                          exp_dir);
  }

  if (tracik_enable)
  {
    std::cerr << "[WARN] " << "正在对 TRAC-IK 运行随机配置测试" << std::endl;
    solveIkFromRandomList(urdf_path, qinit_path, target_path, n_random_config,
                          &trac_ik, "link_ee", (print_all & 2) >> 1, robot_id,
                          exp_dir);
  }

  std::cerr << "[WARN] " << "正在对 COIKS 运行随机配置测试" << std::endl;
  solveIkFromRandomList(urdf_path, qinit_path, target_path, n_random_config,
                        &coiks, "link_ee", (print_all & 4) >> 2, robot_id,
                        exp_dir);
  std::cout << "[INFO] 基准测试完成。结果目录: " << exp_dir << std::endl;

  return 0;
}
