
#ifndef BASE_BOUNDED_QUEUE_H_
#define BASE_BOUNDED_QUEUE_H_

#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <memory>

#include "backend/macros.h"
#include "backend/wait_strategy.h"
#include <iostream>



template <typename T>
class BoundedQueue {
 public:
  using value_type = T;
  using size_type = uint64_t;

 public:
  BoundedQueue() {}
  BoundedQueue& operator=(const BoundedQueue& other) = delete;
  BoundedQueue(const BoundedQueue& other) = delete;
  ~BoundedQueue();
  bool Init(uint64_t size);
  bool Init(uint64_t size, WaitStrategy* strategy, WaitStrategy* strategy2);
  bool Enqueue(const T& element);
  bool Enqueue(T&& element);
  bool WaitEnqueue(const T& element);
  bool WaitEnqueue(T&& element);
  bool Dequeue(T* element);
  bool WaitDequeue(T* element);
  uint64_t Size();
  bool Empty();
  void SetWaitStrategy(WaitStrategy* WaitStrategy);
  void BreakAllWait();
  uint64_t Head() { return head_.load(); }
  uint64_t Tail() { return tail_.load(); }
  uint64_t Commit() { return commit_.load(); }

 private:
  uint64_t GetIndex(uint64_t num);

  alignas(CACHELINE_SIZE) std::atomic<uint64_t> head_ = {0};
  alignas(CACHELINE_SIZE) std::atomic<uint64_t> tail_ = {1};
  alignas(CACHELINE_SIZE) std::atomic<uint64_t> commit_ = {1};
  // alignas(CACHELINE_SIZE) std::atomic<uint64_t> size_ = {0};
  uint64_t pool_size_ = 0;
  T* pool_ = nullptr;
  std::unique_ptr<WaitStrategy> wait_strategy_ = nullptr;
  std::unique_ptr<WaitStrategy> enq_wait_strategy_  =  nullptr;
  volatile bool break_all_wait_ = false;
};

template <typename T>
BoundedQueue<T>::~BoundedQueue() {
  if (wait_strategy_) {
    BreakAllWait();
  }
  if (pool_) {
    for (uint64_t i = 0; i < pool_size_; ++i) {
      pool_[i].~T();
    }
    std::free(pool_);
  }
}

template <typename T>
inline bool BoundedQueue<T>::Init(uint64_t size) {
  return Init(size, new SleepWaitStrategy());
}

template <typename T>
bool BoundedQueue<T>::Init(uint64_t size, WaitStrategy* strategy, WaitStrategy* stategy2) {
  // Head and tail each occupy a space
  pool_size_ = size + 2;
  pool_ = reinterpret_cast<T*>(std::calloc(pool_size_, sizeof(T)));
  if (pool_ == nullptr) {
    return false;
  }
  for (uint64_t i = 0; i < pool_size_; ++i) {
    new (&(pool_[i])) T();
  }
  wait_strategy_.reset(strategy);
  enq_wait_strategy_.reset(stategy2);
  return true;
}

template <typename T>
bool BoundedQueue<T>::Enqueue(const T& element) {
  uint64_t new_tail = 0;
  uint64_t old_commit = 0;
  uint64_t old_tail = tail_.load(std::memory_order_acquire);
  do {
    new_tail = old_tail + 1;
    if (GetIndex(new_tail) == GetIndex(head_.load(std::memory_order_acquire))) {
      return false;
    }
  } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  pool_[GetIndex(old_tail)] = element;
  do {
    old_commit = old_tail;
  } while (vio_unlikely(!commit_.compare_exchange_weak(
      old_commit, new_tail, std::memory_order_acq_rel,
      std::memory_order_relaxed)));
  wait_strategy_->NotifyOne();
  return true;
}

template <typename T>
bool BoundedQueue<T>::Enqueue(T&& element) {
  uint64_t new_tail = 0;
  uint64_t old_commit = 0;
  uint64_t old_tail = tail_.load(std::memory_order_acquire);
  do {
    new_tail = old_tail + 1;
    if (GetIndex(new_tail) == GetIndex(head_.load(std::memory_order_acquire))) {
      return false;
    }
  } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  pool_[GetIndex(old_tail)] = element;
  do {
    old_commit = old_tail;
  } while (vio_unlikely(!commit_.compare_exchange_weak(
      old_commit, new_tail, std::memory_order_acq_rel,
      std::memory_order_relaxed)));
  wait_strategy_->NotifyOne();
  return true;
}

template <typename T>
bool BoundedQueue<T>::Dequeue(T* element) {
  uint64_t new_head = 0;
  uint64_t old_head = head_.load(std::memory_order_acquire);
  do {
    new_head = old_head + 1;
    if (new_head == commit_.load(std::memory_order_acquire)) {
      return false;
    }
    *element = pool_[GetIndex(new_head)];
  } while (!head_.compare_exchange_weak(old_head, new_head,
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  enq_wait_strategy_->NotifyOne();
  return true;
}

template <typename T>
bool BoundedQueue<T>::WaitEnqueue(const T& element) {
  bool ret = false;
  while (!break_all_wait_) {
    auto pred = [this,&element]()->bool {
      return Enqueue(element);
    };
    if(enq_wait_strategy_->PredWait(pred)) {
      // enq succ.
      //std::cout << "Enq succ." << std::endl;
      ret = true;
      break;

    } else {
      // for timeout wait, we may return false.
      // hence continue the loop here.
      std::cerr << "Enq wait timeout" << std::endl;
      continue;
    }
    
  }

  return ret;
}

template <typename T>
bool BoundedQueue<T>::WaitEnqueue(T&& element) {
  auto ret = false;
  while (!break_all_wait_) {
    
    auto pred = [this,&element]()->bool {
      return Enqueue(element);
    };
    if(enq_wait_strategy_->PredWait(pred)) {
      // enq succ.
      //std::cout << "Enq succ" << std::endl;
      ret = true;
      break;
    } else {
      // for timeout wait, we may return false.
      // hence continue the loop here.
      std::cerr << "Enq wait timeout" << std::endl;
      continue;
    }
    //if (Enqueue(element)) {
    //  return true;
    //}
    //if (enq_wait_strategy_->EmptyWait()) {
    //  continue;
    //}
    
    
  }

  return ret;
}

template <typename T>
bool BoundedQueue<T>::WaitDequeue(T* element) {
  bool ret = false;
  while (!break_all_wait_) {

    auto pred = [this, element]()->bool {
      return Dequeue(element);
    };
    if (wait_strategy_->PredWait(pred)) {
      // deq succ.
      //std::cout << "deq succ" << std::endl;
      ret = true;
      break;
    } else {
      std::cerr << "deq wait timeout" << std::endl;
      continue;
    }
    
  }

  return ret;
}

template <typename T>
inline uint64_t BoundedQueue<T>::Size() {
  return tail_ - head_ - 1;
}

template <typename T>
inline bool BoundedQueue<T>::Empty() {
  return Size() == 0;
}

template <typename T>
inline uint64_t BoundedQueue<T>::GetIndex(uint64_t num) {
  return num - (num / pool_size_) * pool_size_;  // faster than %
}

template <typename T>
inline void BoundedQueue<T>::SetWaitStrategy(WaitStrategy* strategy) {
  wait_strategy_.reset(strategy);
}

template <typename T>
inline void BoundedQueue<T>::BreakAllWait() {
  break_all_wait_ = true;
  wait_strategy_->BreakAllWait();
  enq_wait_strategy_->BreakAllWait();
}



#endif  // BOUNDED_QUEUE_H_
