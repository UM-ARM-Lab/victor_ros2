// Copyright 2023, ARMLab @ The University of Michigan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <poll.h>
#include <malloc.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/mman.h>

#include "victor_hardware/VictorFRIHardwareInterface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <span>

auto logger = rclcpp::get_logger("VictorFRIHardwareInterface");


int set_thread_cpu_affinity(pid_t pid, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0U;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0U) {
    if ((cpu_bit_mask & 0x1U) > 0) {
      CPU_SET(cpu_cnt, &set);
    }
    cpu_bit_mask = (cpu_bit_mask >> 1U);
    cpu_cnt++;
  }
  return sched_setaffinity(pid, sizeof(set), &set);
}

int set_this_thread_cpu_affinity(uint32_t cpu_bit_mask)
{
  return set_thread_cpu_affinity(getpid(), cpu_bit_mask);
}

void PrefaultAndLockProcessHeapMemory(int64_t process_memory);

struct ProcessMemoryInformation {
  int64_t hard_page_faults = 0;
  int64_t soft_page_faults = 0;
};

/// Get relevant process memory information, namely the number of hard and soft
/// page faults encountered by the current process.
ProcessMemoryInformation GetCurrentProcessMemoryInformation();

void PrefaultAndLockProcessHeapMemory(int64_t process_memory) {
  // First, "lock" all current and future memory used by the process to prevent
  // memory from being paged out.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    char* error_string = strerror(errno);
    const std::string error_msg(error_string);
    throw std::runtime_error(
        "mlockall(MCL_CURRENT | MCL_FUTURE) failed: " + error_msg);
  }
  // Second, configure the behavior of the system allocator.
  // Disable the return of memory to the OS by disabling heap trim caused by
  // calls to free().
  mallopt(M_TRIM_THRESHOLD, -1);
  // Disable the use of mmap() for handling large allocations.
  mallopt(M_MMAP_MAX, 0);
  // Third, allocate all the desired additional memory.
  uint8_t* allocated = static_cast<uint8_t*>(malloc(process_memory));
  if (allocated == nullptr) {
    throw std::runtime_error("Failed to allocate process memory");
  }
  // Fourth, walk through the allocated memory to page it all in.
  for (int64_t idx = 0; idx < process_memory; idx += sysconf(_SC_PAGESIZE)) {
    // Writes to this buffer will page it in.
    allocated[idx] = 0x00;
  }
  // Fifth, free the memory we allocated. Because we disabled the return of
  // memory to the OS, this memory will stay available and paged in.
  free(allocated);
}

ProcessMemoryInformation GetCurrentProcessMemoryInformation() {
  struct rusage our_rusage;
  if (getrusage(RUSAGE_SELF, &our_rusage) != 0) {
    char* error_string = strerror(errno);
    const std::string error_msg(error_string);
    throw std::runtime_error(
        "getrusage(RUSAGE_SELF, &our_rusage) failed: " + error_msg);
  }
  ProcessMemoryInformation memory_info;
  memory_info.hard_page_faults = our_rusage.ru_majflt;
  memory_info.soft_page_faults = our_rusage.ru_minflt;
  return memory_info;
}

namespace victor_hardware
{
  // ------------------------------------------------------------------------------------------
  CallbackReturn VictorFRIHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    //Set up realtime priority. Based on: https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/kuka-driver/kuka_driver.cc

    // // Lock memory to prevent the OS from paging us out.
    // if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    //   perror("mlockall failed");
    //   return CallbackReturn::ERROR;
    // }
    // RCLCPP_INFO(logger, "Locked memory to prevent paging out"); 

    // // Set realtime priority.
    // struct sched_param scheduler_options = {};
    // // 90 is as high as you generally want to go on PREEMPT_RT machines.
    // // Any higher than this and you will have higher priority than the kernel
    // // threads that serve interrupts, and the system will stop responding.
    // scheduler_options.sched_priority = 90;
    // if (sched_setscheduler(0, SCHED_FIFO, &scheduler_options) != 0) {
    //   perror("sched_setscheduler failed");
    //   return CallbackReturn::ERROR;
    // }
    // RCLCPP_INFO(logger, "Got realtime priority");

    // This controller does all 14 joints, 7 on each arm. The ros2 control interface seems to require
    // that the positions/velocities/etc are all stored as contiguous arrays (i.e std::vector) so all
    // these vectors are of size 14. But of course each arm has its own FRI interface that only understands
    // 7 joints, so we use std::span to create views into the vectors that only contain the relevant joints, 
    // which shouldn't do any copying.
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_external_torque_sensor_.resize(
        info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
    internal_command_position.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // victorRobot has currently exactly 3 state and 1 command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }
      if (hw_command_mode_.empty())
      {
        hw_command_mode_ = joint.command_interfaces[0].name;

        if (hw_command_mode_ != hardware_interface::HW_IF_POSITION &&
            hw_command_mode_ != hardware_interface::HW_IF_VELOCITY &&
            hw_command_mode_ != hardware_interface::HW_IF_EFFORT)
        {
          RCLCPP_FATAL(
              logger,
              "Joint '%s' have %s unknown command interfaces.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str());
          return CallbackReturn::ERROR;
        }
      }

      if (hw_command_mode_ != joint.command_interfaces[0].name)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' has %s command interfaces. Expected %s.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hw_command_mode_.c_str());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' has %li state interface. 3 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }
      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
      if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(
            logger,
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }
  // ------------------------------------------------------------------------------------------
  std::vector<hardware_interface::StateInterface>
  VictorFRIHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    }
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
    }
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name,
              &hw_states_external_torque_sensor_[i]));
    }

    return state_interfaces;
  }
  // ------------------------------------------------------------------------------------------
  std::vector<hardware_interface::CommandInterface>
  VictorFRIHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      if (hw_command_mode_ == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      }
      else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      }
      else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT)
      {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
      }
    }

    return command_interfaces;
  }
  // ------------------------------------------------------------------------------------------
  CallbackReturn VictorFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(logger, "Starting ...please wait...");

    std::string left_ip = info_.hardware_parameters["left_robot_ip"];
    int left_port = stoi(info_.hardware_parameters["left_robot_port"]);
    std::string right_ip = info_.hardware_parameters["right_robot_ip"];
    int right_port = stoi(info_.hardware_parameters["right_robot_port"]);
    leftRobotClient_.setVelocityFilterCutOffFreq(
        stod(
            info_.hardware_parameters["velocity_filter_cutoff_freq"]));
    leftRobotClient_.setTorqueFilterCutOffFreq(
        stod(
            info_.hardware_parameters["torque_filter_cutoff_freq"]));
    rightRobotClient_.setVelocityFilterCutOffFreq(
        stod(
            info_.hardware_parameters["velocity_filter_cutoff_freq"]));
    rightRobotClient_.setTorqueFilterCutOffFreq(
        stod(
            info_.hardware_parameters["torque_filter_cutoff_freq"]));

    // set default value for sensor
    for (auto i = 0ul; i < hw_states_external_torque_sensor_.size(); i++)
    {
      if (std::isnan(hw_states_external_torque_sensor_[i]))
      {
        hw_states_external_torque_sensor_[i] = 0;
      }
    }

    RCLCPP_INFO(logger, "===================================================================================");
    RCLCPP_INFO(logger, "Waiting to connect to FRI, please start the iiwa_ros2 application on BOTH pendants!");
    RCLCPP_INFO(logger, "===================================================================================");
    RCLCPP_INFO(logger, "Left: ip= %s port= %i", left_ip.c_str(), left_port);
    RCLCPP_INFO(logger, "Right: ip= %s port= %i", right_ip.c_str(), right_port);

    leftRobotClient_.connect(left_port, left_ip.c_str());
    rightRobotClient_.connect(right_port, right_ip.c_str());

    RCLCPP_INFO(logger, "On Activate finished successfully");

    return CallbackReturn::SUCCESS;
  }
  // ------------------------------------------------------------------------------------------
  CallbackReturn VictorFRIHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(logger, "Stopping ...please wait...");

    leftRobotClient_.disconnect();
    rightRobotClient_.disconnect();

    RCLCPP_INFO(logger, "System successfully stopped!");

    return CallbackReturn::SUCCESS;
  }
  // ------------------------------------------------------------------------------------------
  hardware_interface::return_type VictorFRIHardwareInterface::read(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/)
  {
    // read FRI and copy positions to hw_states_
    auto const left_ready = leftRobotClient_.updateFromRobot();
    auto const right_ready = rightRobotClient_.updateFromRobot();
    if (!(left_ready && right_ready))
    {
      return hardware_interface::return_type::ERROR;
    }

    auto const left_state = leftRobotClient_.getCurrentControllerState();
    auto const right_state = rightRobotClient_.getCurrentControllerState();
    auto const left_read = left_state != KUKA::FRI::IDLE && left_state != KUKA::FRI::MONITORING_WAIT;
    auto const right_read = right_state != KUKA::FRI::IDLE && right_state != KUKA::FRI::MONITORING_WAIT;
    if (left_read && right_read)
    {
      std::span<double> hw_states_position_span{hw_states_position_};
      auto left_hw_states_position = hw_states_position_span.subspan(0, 7);
      auto right_hw_states_position = hw_states_position_span.subspan(7, 7);
      leftRobotClient_.getRobotJointPosition(left_hw_states_position);
      rightRobotClient_.getRobotJointPosition(right_hw_states_position);

      std::span<double> hw_states_velocity_span{hw_states_velocity_};
      auto left_hw_states_velocity = hw_states_velocity_span.subspan(0, 7);
      auto right_hw_states_velocity = hw_states_velocity_span.subspan(7, 7);
      leftRobotClient_.getRobotJointVelocity(left_hw_states_velocity);
      rightRobotClient_.getRobotJointVelocity(right_hw_states_velocity);

      std::span<double> hw_states_effort_span{hw_states_effort_};
      auto left_hw_states_effort = hw_states_effort_span.subspan(0, 7);
      auto right_hw_states_effort = hw_states_effort_span.subspan(7, 7);
      leftRobotClient_.getRobotJointTorque(left_hw_states_effort);
      rightRobotClient_.getRobotJointTorque(right_hw_states_effort);

      std::span<double> hw_states_et_sensor_span{hw_states_external_torque_sensor_};
      auto left_hw_states_et_sensor = hw_states_et_sensor_span.subspan(0, 7);
      auto right_hw_states_et_sensor = hw_states_et_sensor_span.subspan(7, 7);
      leftRobotClient_.getRobotJointExternalTorque(left_hw_states_et_sensor);
      rightRobotClient_.getRobotJointExternalTorque(right_hw_states_et_sensor);

      if (internal_command_position[0] != internal_command_position[0])
      {
        internal_command_position = hw_states_position_;
      }
    }
    // else
    // {
    //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("VictorFRIHardwareInterface"),  "Not ready to read: " << left_read << " " << right_read << " <<<<<<<");
    // }

    return hardware_interface::return_type::OK;
  }
  // ------------------------------------------------------------------------------------------
  hardware_interface::return_type VictorFRIHardwareInterface::write(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/)
  {
    // write hw_commands_ to FRI
    bool isNan = false;
    for (auto i = 0ul; i < hw_commands_.size(); i++)
    {
      if (std::isnan(hw_commands_[i]))
      {
        isNan = true;
      }
    }

    auto const left_state = leftRobotClient_.getCurrentControllerState();
    auto const right_state = rightRobotClient_.getCurrentControllerState();
    auto const left_active = left_state == KUKA::FRI::COMMANDING_ACTIVE;
    auto const right_active = right_state == KUKA::FRI::COMMANDING_ACTIVE;
    if (left_active && right_active && !isNan)
    {
      if (hw_command_mode_ == hardware_interface::HW_IF_POSITION)
      {
        std::span<double> hw_commands_span{hw_commands_}; // 14
        auto const left_hw_commands = hw_commands_span.subspan(0, 7);
        auto const right_hw_commands = hw_commands_span.subspan(7, 7);
        leftRobotClient_.setTargetJointPosition(left_hw_commands);
        rightRobotClient_.setTargetJointPosition(right_hw_commands);
      }
      else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY)
      {
        for (auto i = 0ul; i < 7; i++)
        {
          internal_command_position[i] = internal_command_position[i] + leftRobotClient_.getRobotStatus().sampleTime * hw_commands_[i];
        }
        for (auto i = 7ul; i < 14; i++)
        {
          internal_command_position[i] = internal_command_position[i] + rightRobotClient_.getRobotStatus().sampleTime * hw_commands_[i];
        }
        std::span<double> internal_command_position_span{internal_command_position};
        auto left_internal_command_position = internal_command_position_span.subspan(0, 7);
        auto right_internal_command_position = internal_command_position_span.subspan(7, 7);
        leftRobotClient_.setTargetJointPosition(left_internal_command_position);
        rightRobotClient_.setTargetJointPosition(right_internal_command_position);
      }
      else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT)
      {
        // in EFFORT mode, hw_commands_ is used for torque and hw_states_position_ for position
        std::span<double> hw_commands_span{hw_commands_};
        auto const left_hw_commands = hw_commands_span.subspan(0, 7);
        auto const right_hw_commands = hw_commands_span.subspan(7, 7);
        leftRobotClient_.setTargetJointTorque(left_hw_commands);
        rightRobotClient_.setTargetJointTorque(right_hw_commands);

        std::span<double> hw_states_position_span{hw_states_position_};
        auto const left_hw_states_position = hw_states_position_span.subspan(0, 7);
        auto const right_hw_states_position = hw_states_position_span.subspan(7, 7);
        leftRobotClient_.setTargetJointPosition(left_hw_states_position);
        rightRobotClient_.setTargetJointPosition(right_hw_states_position);
      }
    }

    auto const left_ok = leftRobotClient_.updateToRobot();
    auto const right_ok = rightRobotClient_.updateToRobot();
    if (!(left_ok && right_ok))
    {
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace victor_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    victor_hardware::VictorFRIHardwareInterface, hardware_interface::SystemInterface)
