#include <iostream>
#include <stack>
#include <drake/systems/plants/RigidBodySystem.h>

using namespace std;
using namespace Eigen;
using namespace Drake;

/**
 * Matlab-like tic toc for benchmarking
 * From: http://stackoverflow.com/a/13485583
 */
std::stack<clock_t> tictoc_stack;
void tic() { tictoc_stack.push(clock()); }
void toc() {
  std::cout << "Time elapsed: "
            << static_cast<double>((clock() - tictoc_stack.top())) /
                   CLOCKS_PER_SEC << std::endl;
  tictoc_stack.pop();
}

int main(int argc, char* argv[]) {
  tic();
  std::string urdf = "/home/wxm/oh-distro/software/models/val_description/urdf/valkyrie_sim_drake.urdf";
                    // getDrakePath() +
                    //  "/examples/Valkyrie/urdf/urdf/"
                    //  "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  Drake::RigidBodySystem rigid_body_sys = Drake::RigidBodySystem();
  rigid_body_sys.addRobotFromFile(urdf, DrakeJoint::FIXED);
  auto const& tree = rigid_body_sys.getRigidBodyTree();
  toc();

  std::cout << "RigidBodySystem has " << rigid_body_sys.getNumStates()
            << " states and " << rigid_body_sys.getNumInputs() << " inputs"
            << std::endl;

  VectorXd x0(rigid_body_sys.getNumStates());

  VectorXd u(rigid_body_sys.getNumInputs());
  u.setZero();
  for (int i = 0; i < rigid_body_sys.getNumInputs(); i++) u(i) = 1.0;

  default_random_engine engine;
  tic();
  for (int i = 0; i < 1000; i++) {
    x0.head(rigid_body_sys.getNumInputs()) =
        tree->getRandomConfiguration(engine);
    auto xdot = rigid_body_sys.dynamics(0.0, x0, u);
  }
  toc();

  return 0;
}
