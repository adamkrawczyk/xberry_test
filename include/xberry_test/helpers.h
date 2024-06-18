#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <thread>

// Random publish time (0.2-5s)
int randomTimer() {
  // Seed the random number generator with the current time
  std::srand(std::time(nullptr) +
             std::hash<std::thread::id>()(std::this_thread::get_id()));
  return 1000 * (0.2 + static_cast<float>(std::rand()) /
                           (static_cast<float>(RAND_MAX / (5 - 0.2))));
}