#include <thread>
#include <iostream>

#include "backend/thread_pool.h"

using std::cout;
using std::endl;



//TODO, there is `'std::bad_function_call'` bug , may change
void thread_pool_block() {
    // q_size is 1.
    ThreadPool pool(2,1);
    int a = 0;
    int b = 0;
    int times = 100;
    std::atomic<int> n_edge  ;
    n_edge.exchange(times);

    auto start = std::chrono::system_clock::now();
    auto t1 = [start, &a, &n_edge]() {
        
        cout << "this is thread 1" << endl;
         auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        cout << "th1 Current time span from start:  " << elapsed_seconds.count() << std::endl;

        std::this_thread::sleep_for(std::chrono::microseconds(10));
        cout << "th1 work done for a: " << a << std::endl;
        n_edge.fetch_sub(1);
        a++;

    };

    auto t2 = [start,&b]() {
        cout << "this is thread 2" << endl;
         auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        cout << "th2 Current time span from start:  " << elapsed_seconds.count() << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        cout << "th2 work done for b: " << b << std::endl;
        b++;
    };


    for(int i=0;i<times;i++) {
        pool.Enqueue(t1);
        pool.Enqueue(t2);
    }
    cout << "Enq done.------------------" << endl;
    while(n_edge.load()>0)
      ;
    
}

int main() {
    thread_pool_block();
}

/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/*#include "gtest/gtest.h"



#include "backend/concurrent_queue.h"
#include "backend/thread_pool.h"



// Just use the protobuf Closure.
using google::protobuf::Closure;
using google::protobuf::NewCallback;

using std::vector;

void Callback(int tag, BlockingCounter* counter) {
  cout << "callback executed, tag: " << tag;
  counter->Decrement();
}

class MyCallback {
 public:
  void Callback(int tag, BlockingCounter* counter) {
    cout << "MyCallback::callback executed, tag: " << tag;
    counter->Decrement();
  }
};

TEST(ThreadPoolTest, Test) {
  ThreadPool thread_pool(2);
  thread_pool.Start();

  BlockingCounter counter(10);
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure = NewCallback(&Callback, idx, &counter);
    thread_pool.Add(closure);
  }

  counter.Wait();
  EXPECT_EQ(thread_pool.num_workers(), 2);
  EXPECT_EQ(thread_pool.num_available_workers(), 2);

  MyCallback my_callback;
  counter.Reset(10);
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure =
        NewCallback(&my_callback, &MyCallback::Callback, idx, &counter);
    thread_pool.Add(closure);
  }

  counter.Wait();

  counter.Reset(10);
  vector<Closure*> closures;
  for (int idx = 0; idx < 10; ++idx) {
    Closure* closure =
        NewCallback(&my_callback, &MyCallback::Callback, idx, &counter);
    closures.push_back(closure);
  }
  thread_pool.Add(closures);
  counter.Wait();

  EXPECT_EQ(thread_pool.num_workers(), 2);
  EXPECT_EQ(thread_pool.num_available_workers(), 2);
}


int main() {
    RUN_ALL_TESTS();
}
*/