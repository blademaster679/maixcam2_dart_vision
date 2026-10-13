#include "dart/latest_worker.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <future>
#include <stdexcept>
#include <chrono>
int main() {
 using namespace std::chrono_literals;
 std::promise<void> started, release, last_started;
 auto gate=release.get_future().share();
 {
  dart::LatestWorker<int,int> worker([&](int n){if(n==1){started.set_value();gate.wait();}if(n==3)last_started.set_value();assert(n!=2);return n;});
  worker.submit(1);assert(started.get_future().wait_for(2s)==std::future_status::ready);
  assert(!worker.take()); // inference does not block the consumer
  worker.submit(2);worker.submit(3);release.set_value();
  auto final_started=last_started.get_future();
  // The first result must be read before another inference can start.
  assert(final_started.wait_for(10ms)==std::future_status::timeout);
  auto deadline=std::chrono::steady_clock::now()+2s;bool got=false;
  while(std::chrono::steady_clock::now()<deadline){auto r=worker.take();if(r&&*r==3){got=true;break;}std::this_thread::yield();}
  assert(got);
  assert(final_started.wait_for(2s)==std::future_status::ready);
 }
 {
  dart::LatestWorker<int,int> worker([](int)->int{throw std::runtime_error("worker failure");});worker.submit(1);
  auto deadline=std::chrono::steady_clock::now()+2s;bool caught=false;
  while(std::chrono::steady_clock::now()<deadline){try{worker.take();}catch(const std::runtime_error&){caught=true;break;}std::this_thread::yield();}
  assert(caught);
 }
}
