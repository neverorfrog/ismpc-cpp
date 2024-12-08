#include <chrono>
#include <iostream>

int main() {
  auto start = std::chrono::high_resolution_clock::now();
  for (int k = 0; k < 100; ++k) {
    std::cout << "Iteration: " << k << std::endl;
  }
  auto end = std::chrono::high_resolution_clock::now();

  return 0;
}
