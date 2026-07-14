/**
 * @file thread_pool.h
 * @author Jakob Progsch
 * @brief 提供固定工作线程数量、FIFO 任务队列和 future 返回值的轻量线程池。
 * @version 0.1
 * @date 2019-07-21
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_
#define _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>

namespace common {
/**
 * @brief 管理固定数量工作线程并异步执行入队任务。
 *
 * 该实现来源于 progschj/ThreadPool。任务所有权由队列持有，析构时停止接收新任务、
 * 唤醒全部线程，并等待队列中已存在的任务执行完毕。
 */
class ThreadPool {
 public:
  /// 创建指定数量的工作线程；线程数为零时任务不会被执行。
  ThreadPool(size_t);

  /**
   * @brief 将可调用对象及其参数绑定为无参任务并放入 FIFO 队列。
   * @return 与任务返回类型一致的 future，用于获取结果或传播异常。
   * @throws std::runtime_error 在线程池已进入停止状态后调用时抛出。
   */
  template <class F, class... Args>
  auto Enqueue(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;
  /// 停止接收任务、唤醒工作线程并等待所有线程退出。
  ~ThreadPool();

 private:
  // 保存线程句柄，析构时必须逐一 join，避免后台线程访问已释放对象。
  std::vector<std::thread> workers;
  // 受 queue_mutex 保护的 FIFO 任务队列。
  std::queue<std::function<void()> > tasks;

  // 条件变量负责在新任务或停止事件到来时唤醒工作线程。
  std::mutex queue_mutex;
  std::condition_variable condition;
  bool stop;
};

// 构造阶段立即启动工作线程；每个线程持续等待任务或停止信号。
inline ThreadPool::ThreadPool(size_t threads) : stop(false) {
  for (size_t i = 0; i < threads; ++i)
    workers.emplace_back([this] {
      for (;;) {
        std::function<void()> task;

        {
          // 仅在取出任务时持锁，实际执行发生在临界区之外。
          std::unique_lock<std::mutex> lock(this->queue_mutex);
          this->condition.wait(
              lock, [this] { return this->stop || !this->tasks.empty(); });
          if (this->stop && this->tasks.empty()) return;
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }

        task();
      }
    });
}

// 模板实现必须放在头文件中，供调用方按具体可调用类型实例化。
template <class F, class... Args>
auto ThreadPool::Enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  // packaged_task 将返回值和异常统一转交给 future。
  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // 停止状态不可逆，禁止继续向即将销毁的线程池提交任务。
    if (stop) throw std::runtime_error("enqueue on stopped ThreadPool");

    tasks.emplace([task]() { (*task)(); });
  }
  condition.notify_one();
  return res;
}

// 先设置停止标记，再广播唤醒，最后等待工作线程完成剩余队列并退出。
inline ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
  }
  condition.notify_all();
  for (std::thread& worker : workers) worker.join();
}
}  // namespace common

#endif  // _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_
