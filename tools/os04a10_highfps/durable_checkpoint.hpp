#pragma once
#include <chrono>
#include <future>
#include <stdexcept>
#include <string>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

// One background sync at a time. Offsets are lower bounds captured after the
// writer flushes both streams; concurrent appends may also reach storage.
class DurableCheckpoint {
    std::future<void> pending;
    struct Fd {
        int value;
        Fd(const char *path,int flags):value(::open(path,flags,0644)) {
            if(value<0)throw std::runtime_error(std::string("checkpoint open ")+path+": "+std::strerror(errno));
        }
        ~Fd(){::close(value);}
        Fd(const Fd&)=delete;
    };
    static void sync_fd(int fd,bool directory=false) {
        int rc;
        do {rc=directory ? ::fsync(fd) : ::fdatasync(fd);} while(rc && errno==EINTR);
        if(rc)throw std::runtime_error(std::string("checkpoint sync: ")+std::strerror(errno));
    }
public:
    struct Snapshot { unsigned long long video_bytes,index_bytes,frames,elapsed_ms; };
    static void sync(Snapshot s) {
        Fd video("record.h264",O_RDWR),index("encoded_frames.csv",O_RDWR);
        sync_fd(video.value);sync_fd(index.value);
        std::string text="{\"video_bytes\":"+std::to_string(s.video_bytes)+
            ",\"index_bytes\":"+std::to_string(s.index_bytes)+",\"frames\":"+std::to_string(s.frames)+
            ",\"requested_elapsed_ms\":"+std::to_string(s.elapsed_ms)+"}\n";
        Fd checkpoint("durable_checkpoint.json.tmp",O_WRONLY|O_CREAT|O_TRUNC);
        size_t offset=0;
        while(offset<text.size()) {
            auto n=::write(checkpoint.value,text.data()+offset,text.size()-offset);
            if(n<0 && errno==EINTR)continue;
            if(n<=0)throw std::runtime_error("checkpoint write failed");
            offset+=static_cast<size_t>(n);
        }
        sync_fd(checkpoint.value);
        if(::rename("durable_checkpoint.json.tmp","durable_checkpoint.json"))
            throw std::runtime_error("checkpoint rename failed");
        Fd directory(".",O_RDONLY|O_DIRECTORY);sync_fd(directory.value,true);
    }
    bool ready() {
        if(!pending.valid())return true;
        if(pending.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return false;
        pending.get();return true; // propagate background I/O failure
    }
    bool request(Snapshot s) {
        if(!ready())return false;
        pending=std::async(std::launch::async,[s]{sync(s);});return true;
    }
    void wait(){if(pending.valid())pending.get();}
    ~DurableCheckpoint(){if(pending.valid())pending.wait();}
};
