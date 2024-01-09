#include <string.h>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <poll.h>
#include <malloc.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/mman.h>

#include <victor_hardware/IRDFClient.h>

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

int main() {

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

  KUKA::FRI::IRDFClient left_client;
  KUKA::FRI::IRDFClient right_client;

  left_client.connect(30200, "192.170.12.1");
  // right_client.connect(30201, "192.170.11.1");
  std::cout << "Connected" << std::endl;
  left_client.setVelocityFilterCutOffFreq(40);
  left_client.setTorqueFilterCutOffFreq(40);
  // right_client.setVelocityFilterCutOffFreq(40);
  // right_client.setTorqueFilterCutOffFreq(40);

  auto t = 0;
  auto const t0 = std::chrono::steady_clock::now();
  while (true) {
    auto const rw0 = std::chrono::high_resolution_clock::now();
    auto const left_ready = left_client.updateFromRobot();
    auto const rw1 = std::chrono::high_resolution_clock::now();
    auto const left_update_ok = left_client.updateToRobot();
    auto const rw2 = std::chrono::high_resolution_clock::now();
    auto const rw_dt1 = std::chrono::duration_cast<std::chrono::microseconds>(rw1 - rw0);
    auto const rw_dt2 = std::chrono::duration_cast<std::chrono::microseconds>(rw2 - rw1);

    auto const now = std::chrono::steady_clock::now();
    // print the time since the start in minutes, stop after 1 hour
    auto const dt_min = std::chrono::duration_cast<std::chrono::minutes>(now - t0);

    // usleep(3000);

    if (t % 500 == 0) {
      std::cout << t << " "
                << "rw: dt 1 " << rw_dt1.count() << "us "
                << "rw: dt 2 " << rw_dt2.count() << "us "
                // << "total dt min: " << dt_min.count() << " "
                // << "status: "
                // << left_ready << " "
                // << right_ready << " "
                // << left_update_ok << " "
                // << right_update_ok
                << std::endl;
    }

    t += 1;

    if (dt_min.count() > 60) {
      std::cout << "Done!" << std::endl;
      break;
    }
  }

  std::cout << "Disconnecting" << std::endl;

  left_client.disconnect();
  right_client.disconnect();

  std::cout << "Disconnected" << std::endl;

  return 0;
}