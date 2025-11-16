#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SafeQueue {
public:
    void push(const T& item) {
        {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(item);
        }
        cond_var_.notify_one();
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this]{ return !queue_.empty(); });

        T value = queue_.front();
        queue_.pop();
        return value;
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void insert_and_cleanup(std::map<int64_t, T>& buffer, std::mutex& mtx, 
                        const T& data, int64_t stamp_ns)
    {
        std::lock_guard<std::mutex> lock(mtx);

        buffer[stamp_ns] = data;

        // keep last 200 samples max
        if (buffer.size() > 200) {
            buffer.erase(buffer.begin());
        }
    }

    bool find_nearest(const std::map<int64_t, T>& buffer, int64_t target_ns, 
                  T& out_data, int64_t max_diff_ns)
    {
        if (buffer.empty()) return false;

        auto it = buffer.lower_bound(target_ns);

        // Check candidate 1 — the first entry >= target
        int64_t best_diff = INT64_MAX;

        if (it != buffer.end()) {
            int64_t diff = std::abs(it->first - target_ns);
            if (diff < best_diff) {
                best_diff = diff;
                out_data = it->second;
            }
        }

        // Check candidate 2 — previous entry < target
        if (it != buffer.begin()) {
            auto prev = std::prev(it);
            int64_t diff = std::abs(prev->first - target_ns);
            if (diff < best_diff) {
                best_diff = diff;
                out_data = prev->second;
            }
        }

        return best_diff <= max_diff_ns;
    }



private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_var_;    


};