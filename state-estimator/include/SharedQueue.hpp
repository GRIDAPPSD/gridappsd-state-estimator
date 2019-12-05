//
// Copyright (c) 2013 Juan Palacios juan.palacios.puyana@gmail.com
// Subject to the BSD 2-Clause License
// - see < http://opensource.org/licenses/BSD-2-Clause>
//

#ifndef SHAREDQUEUE_HPP
#define SHAREDQUEUE_HPP

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class SharedQueue
{
 public:

  T pop() 
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto val = queue_.front();
    queue_.pop();
    return val;
  }

  void pop(T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.front();
    queue_.pop();
  }

  void push(const T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }

  int size()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    int size = queue_.size();
    mlock.unlock();
    return size;
  }

  bool empty()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    bool empty = queue_.empty();
    mlock.unlock();
    return empty;
  }

  SharedQueue()=default;
  SharedQueue(const SharedQueue&) = delete;            // disable copying
  SharedQueue& operator=(const SharedQueue&) = delete; // disable assignment
  
 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#endif
