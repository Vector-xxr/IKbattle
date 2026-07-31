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
    
    KDL::JntArray q_init_kdl(n_joints_);
    for(int i=0; i<n_joints_; ++i) q_init_kdl(i) = q_init(i);

    rc = tracik_solver_->CartToJnt(q_init_kdl, ee, qd);

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

void solveIkPath(std::string urdf_file_, std::string qinit_file_,
                 std::string path_file_, int point_id, IkSolver *ik_solver,
                 std::string ee_link_, bool print_all, int robot_id,
                 const std::string &output_dir)
{
  double                  total_time = 0;
  double                  total_time_it = 0;
  double                  success_time_it = 0;
  uint                    n_success = 0;
  IKSolutionSet::Solution bestSolution;

  pin::Model      model_;
  pin::Data       mdl_data_;
  pin::FrameIndex ee_id_;

  VectorXd q_sol_;
  int n_joints_;

  std::fstream fout;

  pin::urdf::buildModel(urdf_file_, model_);
  ee_id_ = model_.getFrameId(ee_link_);

  n_joints_ = model_.nq;
  q_sol_.resize(n_joints_);

  std::cout << "使用求解器求解 IK 路径: " << ik_solver->get_solver_name()
            << "，机器人 " << robot_id << "，目标链接 " << ee_link_ << std::endl;

  std::vector<pin::SE3> xdes_list;
  VectorXd q_init(ik_solver->get_n_joints());

  // Read qinit value from CSV file
  std::ifstream qinit_file(qinit_file_);
  std::string line;
  if (qinit_file.is_open()) {
    std::getline(qinit_file, line); // Skip header
    int current_line = 0;
    while (std::getline(qinit_file, line)) {
        if (current_line == point_id) {
            std::stringstream linestream(line);
            std::string value;
            std::getline(linestream, value, ','); // Skip problem_id
            for (int i = 0; i < q_init.size(); ++i) {
                std::getline(linestream, value, ',');
                q_init(i) = std::stod(value);
            }
            break;
        }
        current_line++;
    }
    qinit_file.close();
  } else {
    std::cerr << "无法打开文件: " << qinit_file_ << std::endl;
    return;
  }

  // Read path from CSV file
  std::ifstream path_file(path_file_);
  if (path_file.is_open()) {
      std::getline(path_file, line); // Skip header
      while (std::getline(path_file, line)) {
          std::stringstream linestream(line);
          std::string value;
          pin::SE3 xdes;
          for (int i = 0; i < 3; ++i) {
              std::getline(linestream, value, ',');
              xdes.translation()(i) = std::stod(value);
          }
          Eigen::Quaterniond quat;
          std::getline(linestream, value, ','); quat.x() = std::stod(value);
          std::getline(linestream, value, ','); quat.y() = std::stod(value);
          std::getline(linestream, value, ','); quat.z() = std::stod(value);
          std::getline(linestream, value, ','); quat.w() = std::stod(value);
          xdes.rotation() = quat.toRotationMatrix();
          xdes_list.push_back(xdes);
      }
      path_file.close();
  } else {
      std::cerr << "无法打开文件: " << path_file_ << std::endl;
      return;
  }

  const std::string csv_path = makeResultCsvPath(
      output_dir, ik_solver->get_solver_name(), robot_id, "_path");
  fout.open(csv_path, std::ios::out);
  if (!fout.is_open())
  {
    std::cerr << "[ERROR] 无法打开结果文件: " << csv_path << std::endl;
  }
  else
  {
    std::cout << "[INFO] 结果写入: " << csv_path << std::endl;
  }
  fout << "idx,res,time,solver_id,sol_id,error,q_init,q_sol\n";

  VectorXd q_seed = q_init;
  auto start_cb_time = std::chrono::high_resolution_clock::now();

  for (uint i = 0; i < xdes_list.size(); i++) {
    auto start_it_time = std::chrono::high_resolution_clock::now();
    int res = 1;
    if (ik_solver->get_solver_name() == "trac_ik") {
      res = ik_solver->IkSolve(q_seed, xdes_list[i], q_sol_);
    } else {
      res = ik_solver->IkSolve(q_seed, xdes_list[i], q_sol_, bestSolution);
    }
    auto stop_it_time = std::chrono::high_resolution_clock::now();
    auto duration_it = std::chrono::duration_cast<std::chrono::microseconds>(stop_it_time - start_it_time);
    total_time_it += duration_it.count();

    if (!res) {
      success_time_it += duration_it.count();
      n_success++;
      q_seed = q_sol_; // Use successful solution as seed for next iteration
      if (ik_solver->get_solver_name() != "trac_ik") {
          fout << i << "," << res << "," << duration_it.count() << ","
               << bestSolution.solver_id << "," << bestSolution.solution_id << ","
               << bestSolution.metrics["error"] << "," << q_init.transpose()
               << "," << bestSolution.configuration.transpose() << "\n";
      } else {
          fout << i << "," << res << "," << duration_it.count() << ",,,,"
               << q_init.transpose() << "," << q_sol_.transpose() << "\n";
      }
    } else {
        fout << i << "," << res << "," << duration_it.count() << ",,,,"
             << q_init.transpose() << "," << q_init.transpose() << "\n";
    }
  }

  auto stop_cb_time = std::chrono::high_resolution_clock::now();
  auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(stop_cb_time - start_cb_time);
  total_time = duration_cb.count();

  std::cout << "------------------------------" << std::endl;
  std::cout << ik_solver->get_solver_name() << " 的汇总:" << std::endl;
  std::cout << "成功率: " << 100.0 * n_success / xdes_list.size() << " %" << std::endl;
  std::cout << "每个路径点平均耗时: " << total_time_it / xdes_list.size() << " 微秒" << std::endl;
  if (n_success > 0) {
    std::cout << "成功路径点平均耗时: " << success_time_it / n_success << " 微秒" << std::endl;
  }
  std::cout << "------------------------------" << std::endl;

  fout.close();
}

bool kill_process = false;
void SigIntHandler(int signal)
{
  kill_process = true;
  std::cout << "[INFO] " << "收到关闭信号" << std::endl;
}

int main(int argc, char **argv)
{
  std::cout << "[INFO] " << "COIKS 与 TRAC-IK 路径跟踪基准测试" << std::endl;
  ros::init(argc, argv, "coiks_benchmark_path");
  ros::NodeHandle nh;
  std::signal(SIGINT, SigIntHandler);

  std::string ik_solver_name;
  nh.param<std::string>("ik_solver", ik_solver_name, "coiks_invj_multi");

  int robot_id;
  nh.param("robot_id", robot_id, 0);

  int point_id;
  nh.param("point_id", point_id, 0);

  double max_time, max_error;
  int max_iter;
  nh.param("max_time", max_time, 0.01);
  nh.param("max_error", max_error, 1e-5);
  nh.param("max_iter", max_iter, 5000);

  bool tracik_enable;
  nh.param("tracik_enable", tracik_enable, true);

  std::string pkg_path = ros::package::getPath("coiks_benchmark");
  std::string urdf_path, qinit_path, path_file;

  std::vector<std::string> robot_names = {
    "hsr", "vs050", "xarm6", "irb140", "jaco", "ur5", "ur10", 
    "gen3", "iiwa", "xarm7", "panda", "sawyer", "yumi", "fetch", "tiago"
  };

  std::string robot_name_str = robot_names[robot_id];

  switch (robot_id)
  {
  case 0:
    urdf_path = pkg_path + std::string("/urdf/") + "toyota/hsr/hsr_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_hsr_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 1:
    urdf_path = pkg_path + std::string("/urdf/") + "denso/vs050/vs050.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_vs050_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 2:
    urdf_path = pkg_path + std::string("/urdf/") + "ufactory/xarm6/xarm6.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_xarm6_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 3:
    urdf_path = pkg_path + std::string("/urdf/") + "abb/irb140/irb140.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_irb140_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 4:
    urdf_path = pkg_path + std::string("/urdf/") + "kinova/jaco/jaco_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_jaco_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 5:
    urdf_path = pkg_path + std::string("/urdf/") + "ur/ur5/ur5.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_ur5_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 6:
    urdf_path = pkg_path + std::string("/urdf/") + "ur/ur10/ur10.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_ur10_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 7:
    urdf_path = pkg_path + std::string("/urdf/") + "kinova/gen3/gen3.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_gen3_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 8:
    urdf_path = pkg_path + std::string("/urdf/") + "kuka/iiwa/iiwa.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_iiwa_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 9:
    urdf_path = pkg_path + std::string("/urdf/") + "ufactory/xarm7/xarm7.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_xarm7_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 10:
    urdf_path = pkg_path + std::string("/urdf/") + "franka/panda.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_panda_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 11:
    urdf_path = pkg_path + std::string("/urdf/") + "rethink/sawyer/sawyer.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_sawyer_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 12:
    urdf_path = pkg_path + std::string("/urdf/") + "abb/yumi/yumi.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_yumi_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 13:
    urdf_path = pkg_path + std::string("/urdf/") + "fetch/fetch_arm.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_fetch_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  case 14:
    urdf_path = pkg_path + std::string("/urdf/") + "tiago/tiago.urdf";
    qinit_path = pkg_path + std::string("/data/") +
                 "qinit_robot_tiago_random_10000samples.csv";
    path_file = pkg_path + "/data/path_robot_" + std::to_string(robot_id) + ".csv";
    break;

  default:
    std::cerr << "[ERROR] " << "无法识别运动学树 ID。" << std::endl;
    return -1;
  }

  std::cout << "[INFO] " << "正在使用 URDF: " << urdf_path << std::endl;
  std::cout << "[INFO] " << "正在使用 qinit 文件: " << qinit_path << std::endl;
  std::cout << "[INFO] " << "正在使用路径文件: " << path_file << std::endl;

  SolverOptions so;
  nh.param<std::string>("solve_mode", so.solve_mode_, "speed");
  nh.param<int>("invj_multi_n_solvers", so.invj_multi_n_solvers_, 4);
  nh.param<std::string>("error_type", so.error_type_, "log6");
  nh.param<std::string>("step_size_method", so.step_size_method_, "fixed");
  nh.param<std::string>("seed_method", so.seed_method_, "current");
  nh.param<std::string>("limiting_method", so.limiting_method_, "random");
  nh.param<std::string>("pinv_method", so.pinv_method_, "cod");
  nh.param<double>("invj_Ke1", so.invj_Ke1_, 1.0);
  nh.param<int>("invj_max_stagnation_iter", so.invj_max_stagnation_iterations_, 10);
  nh.param<double>("invj_improvement_threshold", so.invj_improvement_threshold_, 1e-5);
  double mu0, mu1, mu2;
  nh.param<double>("nlo_mu0", mu0, 1.0);
  nh.param<double>("nlo_mu1", mu1, 1.0);
  nh.param<double>("nlo_mu2", mu2, 1e-7);
  so.cost_coeff_.push_back(mu0);
  so.cost_coeff_.push_back(mu1);
  so.cost_coeff_.push_back(mu2);
  nh.param<bool>("nlo_concurrent", so.nlo_concurrent_, false);
  nh.param<std::string>("nlo_error_type", so.nlo_error_type_, "log6");
  nh.param<int>("nlo_concurrent_iterations", so.nlo_concurrent_iterations_, 3);
  nh.param<std::string>("nlo_warm_start", so.nlo_warm_start_, "yes");
  nh.param<double>("hqp_K_t1", so.qp_K_t1_, 1000.0);
  nh.param<double>("hqp_Kr_t1", so.qp_Kr_t1_, 1.0);
  nh.param<double>("hqp_Kw_p1", so.qp_Kw_p1_, 1e-8);
  nh.param<double>("hqp_Kd_p1", so.qp_Kd_p1_, 1e-8);
  nh.param<bool>("hqp_warm_start", so.qp_warm_start_, true);
  nh.param<std::string>("solv_time_stats", so.time_stats_, "no");
  nh.param<int>("solv_verb_level", so.verb_level_, 0);

  const std::string exp_dir =
      createNextExperimentDir(pkg_path + "/output");
  std::cout << "[INFO] 实验结果目录: " << exp_dir << std::endl;

  if (tracik_enable) {
    std::cout << "[INFO] " << "正在启动 TRAC-IK" << std::endl;
    BenchmarkTRACIK trac_ik(urdf_path, "base_link", "link_ee", ik_solver_name, so.solve_mode_, max_time, max_error, max_iter, 1.0);
    solveIkPath(urdf_path, qinit_path, path_file, point_id, &trac_ik, "link_ee", false, robot_id, exp_dir);
  }

  std::cout << "[INFO] " << "正在启动 COIKS" << std::endl;
  COIKS coiks(urdf_path, "base_link", "link_ee", ik_solver_name, so, max_time, max_error, max_iter, 1.0);
  solveIkPath(urdf_path, qinit_path, path_file, point_id, &coiks, "link_ee", false, robot_id, exp_dir);

  std::cout << "[INFO] 基准测试完成。结果目录: " << exp_dir << std::endl;

  return 0;
}
