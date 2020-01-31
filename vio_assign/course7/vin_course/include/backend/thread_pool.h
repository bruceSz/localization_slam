
#ifndef THREAD_POOL_H_
#define THREAD_POOL_H_

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <queue>
#include <stdexcept>
#include <thread>
#include <utility>
#include <iostream>
#include <atomic>
#include <vector>

#include "backend/bounded_queue.h"

class ThreadPool {
 public:
  explicit ThreadPool(std::size_t thread_num, std::size_t max_task_num = 1000);

  template <typename F, typename... Args>
  auto Enqueue(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;

  //bool Stop();
  ~ThreadPool();

  int NumIdle() {
    return workers_.size() - running_.load();
  }

 private:
  std::vector<std::thread> workers_;
  BoundedQueue<std::function<void()>> task_queue_;
  std::atomic_bool stop_;
  std::atomic<int> running_;
};

inline ThreadPool::ThreadPool(std::size_t threads, std::size_t max_task_num)
    : stop_(false) {
      running_.exchange(0);
  if (!task_queue_.Init(max_task_num, new BlockWaitStrategy(),new BlockWaitStrategy())) {
    throw std::runtime_error("Task queue init failed.");
  }
  workers_.reserve(threads);
  for (size_t i = 0; i < threads; ++i) {
    workers_.emplace_back([this] {
      while (!stop_) {
        std::function<void()> task;
        if (task_queue_.WaitDequeue(&task)) {
          running_.fetch_add(1);
          task();
          running_.fetch_sub(1);
        }
      }
    });
  }
}

// before using the return value, you should check value.valid()
template <typename F, typename... Args>
auto ThreadPool::Enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();

  // don't allow enqueueing after stopping the pool
  if (stop_) {
    return std::future<return_type>();
  }
  auto enq_status = task_queue_.WaitEnqueue([task]() { (*task)(); });
  if( !enq_status) 
    std::cout  << "enque failed " << std::endl;
  return res;
};


// the destructor joins all threads
inline ThreadPool::~ThreadPool() {
  //if (stop_.exchange(true)) {
  //  return;
  //}

  stop_.exchange(true);
  task_queue_.BreakAllWait();
  for (std::thread& worker : workers_) {
    worker.join();
  }
  return ;
}





#endif  // CYBER_BASE_THREAD_POOL_H_
