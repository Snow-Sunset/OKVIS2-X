// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void run() {
  using namespace std::chrono_literals;
  okvis::threadsafe::Queue<int> queue;
  check(queue.PushNonBlockingIfNotFull(1, 1), "First bounded push rejected");
  check(!queue.PushNonBlockingIfNotFull(2, 1), "Overflow push did not reject");
  check(queue.Size() == 1, "Rejected push changed queue size");
  int value = 0;
  check(queue.PopNonBlocking(&value) && value == 1, "Overflow push discarded earlier work");
  queue.PushNonBlockingIfNotFull(3, 1);
  auto producer = std::async(std::launch::async, [&] { return queue.PushBlockingIfFull(4, 1); });
  check(producer.wait_for(20ms) == std::future_status::timeout, "Bounded producer did not wait");
  check(queue.PopNonBlocking(&value) && value == 3, "Nonblocking consumer lost queued work");
  check(producer.wait_for(1s) == std::future_status::ready, "Nonblocking pop did not wake bounded producer");
  check(producer.get(), "Bounded producer failed after space became available");
  check(queue.PopNonBlocking(&value) && value == 4, "Woken producer changed item ordering");
  queue.Shutdown();
  check(!queue.PushNonBlockingIfNotFull(5, 1), "Shutdown queue accepted bounded work");

  okvis::threadsafe::Queue<int> waitingQueue;
  auto consumer = std::async(std::launch::async, [&] {
    int waitingValue = 0;
    return waitingQueue.PopBlocking(&waitingValue);
  });
  check(consumer.wait_for(20ms) == std::future_status::timeout,
        "Blocking consumer returned before data or shutdown");
  waitingQueue.Shutdown();
  check(consumer.wait_for(1s) == std::future_status::ready,
        "Shutdown did not wake blocking consumer");
  check(!consumer.get(), "Shutdown blocking pop reported a value");

  okvis::threadsafe::Queue<int> waitingForDataQueue;
  auto waiter = std::async(std::launch::async, [&] { return waitingForDataQueue.WaitForData(); });
  check(waiter.wait_for(20ms) == std::future_status::timeout,
        "Data waiter returned before data or shutdown");
  waitingForDataQueue.Shutdown();
  check(waiter.wait_for(1s) == std::future_status::ready,
        "Shutdown did not wake data waiter");
  check(!waiter.get(), "Shutdown data wait reported availability");

  okvis::threadsafe::Queue<std::unique_ptr<int>> moveQueue;
  auto moved = std::make_unique<int>(7);
  check(moveQueue.PushNonBlockingIfNotFull(std::move(moved), 1), "Move-only bounded push failed");
  check(!moved, "Move-only bounded push copied its input");
  std::unique_ptr<int> result;
  check(moveQueue.PopNonBlocking(&result) && result && *result == 7,
        "Move-only queue pop lost its payload");
  moveQueue.PushNonBlocking(std::make_unique<int>(8));
  auto moveProducer = std::async(std::launch::async, [&] {
    return moveQueue.PushBlockingIfFull(std::make_unique<int>(9), 1);
  });
  check(moveProducer.wait_for(20ms) == std::future_status::timeout,
        "Move-only bounded producer did not wait");
  check(moveQueue.PopNonBlocking(&result) && result && *result == 8,
        "Move-only blocking test lost the queued payload");
  check(moveProducer.wait_for(1s) == std::future_status::ready && moveProducer.get(),
        "Move-only bounded producer was not released");
  check(moveQueue.PopNonBlocking(&result) && result && *result == 9,
        "Move-only blocking push lost its payload");
}
}
int main() {
  try { run(); std::cout << "Bounded queue rejection and producer wakeup checks passed\n"; }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
