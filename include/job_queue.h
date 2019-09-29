//
// Created by a409 on 9/28/19.
//

#ifndef INDEMIND_COLLECT_DATA_JOBQUEUE_H
#define INDEMIND_COLLECT_DATA_JOBQUEUE_H

#include <atomic>
#include <functional>
#include <list>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <condition_variable>


// A job queue class for the producer-consumer paradigm.
//
//    JobQueue<int> job_queue;
//
//    std::thread producer_thread([&job_queue]() {
//      for (int i = 0; i < 10; ++i) {
//        job_queue.Push(i);
//      }
//    });
//
//    std::thread consumer_thread([&job_queue]() {
//      for (int i = 0; i < 10; ++i) {
//        const auto job = job_queue.Pop();
//        if (job.IsValid()) { /* Do some work */ }
//        else { break; }
//      }
//    });
//
//    producer_thread.join();
//    consumer_thread.join();
//
template<typename T>
class JobQueue {
public:
    class Job {
    public:
        Job() : valid_(false) {}

        explicit Job(const T &data) : data_(data), valid_(true) {}

        // Check whether the data is valid
        bool IsValid() const { return valid_; }

        // Get reference to the data
        T &Data() { return data_; }

        const T &Data() const { return data_; }

    private:
        T data_;
        bool valid_;
    }; // class Job
    JobQueue();

    explicit JobQueue(const size_t max_num_jobs);

    ~JobQueue();

    // The number of pushed and not popped jobs in the queue
    size_t Size();

    // Push a new job to the queue. Waits if the number of jobs is exceeded
    bool Push(const T &data);

    // Pop a job from the queue. Waits if there is no job in the queue.
    Job Pop();

    // Wait for all jobs to be popped and the stop the queue.
    void Wait();

    // Stop the queue and return from all push/pop calls with false
    void Stop();

    // Clear all pushed andn ot popped jobs from the queue.
    void Clear();

private:
    size_t max_num_jobs_;
    std::atomic<bool> stop_;
    std::queue<T> jobs_;
    std::mutex mutex_;
    std::condition_variable push_condition_;
    std::condition_variable pop_condition_;
    std::condition_variable empty_condition_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template<typename T>
JobQueue<T>::JobQueue():JobQueue(std::numeric_limits<size_t>::max()) {}

template<typename T>
JobQueue<T>::JobQueue(const size_t max_num_jobs):max_num_jobs_(max_num_jobs), stop_(false) {}

template<typename T>
JobQueue<T>::~JobQueue() { Stop(); }

template<typename T>
size_t JobQueue<T>::Size() {
    std::unique_lock<std::mutex> lock(mutex_);
    return jobs_.size();
}

template<typename T>
bool JobQueue<T>::Push(const T &data) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (jobs_.size() >= max_num_jobs_ && !stop_) {
        pop_condition_.wait(lock);
    }
    if (stop_) {
        return false;
    } else {
        jobs_.push(data);
        push_condition_.notify_one();
        return true;
    }
}

template<typename T>
typename JobQueue<T>::Job JobQueue<T>::Pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (jobs_.empty() && !stop_) {
        push_condition_.wait(lock);
    }
    if (stop_) {
        return Job();
    } else {
        const T data = jobs_.front();
        jobs_.pop();
        pop_condition_.notify_one();
        if (jobs_.empty()) {
            empty_condition_.notify_all();
        }
        return Job(data);
    }
}

template<typename T>
void JobQueue<T>::Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!jobs_.empty()) {
        empty_condition_.wait(lock);
    }
}

template<typename T>
void JobQueue<T>::Stop() {
    stop_ = true;
    push_condition_.notify_all();
    pop_condition_.notify_all();
}

template<typename T>
void JobQueue<T>::Clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::queue<T> empty_jobs;
    std::swap(jobs_, empty_jobs);
}


#endif //INDEMIND_COLLECT_DATA_JOBQUEUE_H
