#include "statistics.hpp"

#include <chrono>
#include <thread>
#include <mutex>

namespace
{
    void deliberatelySlowIncrement(int &value)
    {
        // This delay makes the race observable on small homework inputs.
        const int old = value;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        value = old + 1;
    }
}

void Statistics::onProduced()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    deliberatelySlowIncrement(produced_);
}

void Statistics::onProcessed()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    deliberatelySlowIncrement(processed_);
}

void Statistics::onSaved()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    deliberatelySlowIncrement(saved_);
}

void Statistics::onCorrupted()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    deliberatelySlowIncrement(corrupted_);
}

StatisticsSnapshot Statistics::snapshot() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return {produced_, processed_, saved_, corrupted_};
}
