#pragma once
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace dart {
// One running job and at most one pending job. Completed results must be
// consumed before another job starts, so a negative confirmation cannot be lost. The caller owns job data
// until destruction joins the worker; use owned buffers or immutable leases.
template<class Job, class Result> class LatestWorker {
 public:
  explicit LatestWorker(std::function<Result(Job)> run): run_(std::move(run)), thread_([this]{loop();}) {}
  ~LatestWorker() { {std::lock_guard<std::mutex> lock(mutex_); stopping_=true; pending_.reset();} cv_.notify_one(); thread_.join(); }
  LatestWorker(const LatestWorker&)=delete;
  LatestWorker& operator=(const LatestWorker&)=delete;
  void submit(Job job) { {std::lock_guard<std::mutex> lock(mutex_); rethrow(); pending_=std::move(job);} cv_.notify_one(); }
  std::optional<Result> take() { std::lock_guard<std::mutex> lock(mutex_); rethrow(); auto result=std::move(result_); result_.reset(); cv_.notify_one(); return result; }
 private:
  void rethrow() {if(error_) std::rethrow_exception(error_);}
  void loop() noexcept {
   try {for(;;) {
    std::optional<Job> job;
    {std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock,[this]{return stopping_||(pending_.has_value()&&!result_.has_value());}); if(stopping_)return; job=std::move(pending_);pending_.reset();}
    auto result=run_(std::move(*job));
    {std::lock_guard<std::mutex> lock(mutex_);result_=std::move(result);}
   }} catch(...) {std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();}
  }
  std::function<Result(Job)> run_;
  std::mutex mutex_; std::condition_variable cv_; bool stopping_=false;
  std::optional<Job> pending_; std::optional<Result> result_; std::exception_ptr error_;
  std::thread thread_;
};
}
