#include "custom_dog_control/precision/LatestSnapshot.hpp"
#include <atomic>
#include <condition_variable>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>

using custom_dog_control::LatestSnapshot;

namespace {
struct Record {
  unsigned sequence = 0;
  std::string text = "0";
};

struct CopyGate {
  std::mutex mutex;
  std::condition_variable cv;
  bool copying = false;
  bool release = false;
};

struct GatedRecord {
  CopyGate *gate = nullptr;
  GatedRecord() = default;
  GatedRecord(const GatedRecord &other) : gate(other.gate) {
    if (gate) {
      std::unique_lock<std::mutex> lock(gate->mutex);
      gate->copying = true;
      gate->cv.notify_all();
      gate->cv.wait(lock, [&] { return gate->release; });
    }
  }
  GatedRecord &operator=(const GatedRecord &) = default;
};
} // namespace

TEST(LatestSnapshot, ReadersReceiveCoherentOwnedCopies) {
  LatestSnapshot<Record> snapshot;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (unsigned i = 1; i <= 10000; ++i)
      snapshot.TryWrite(Record{i, std::to_string(i)});
    done = true;
  });
  do {
    const auto value = snapshot.Read();
    EXPECT_EQ(value.text, std::to_string(value.sequence));
  } while (!done);
  writer.join();
  snapshot.Reset();
  EXPECT_EQ(snapshot.Read().sequence, 0u);
}

TEST(LatestSnapshot, BusyReaderMakesProducerSkipInsteadOfWaiting) {
  LatestSnapshot<GatedRecord> snapshot;
  CopyGate gate;
  GatedRecord value;
  value.gate = &gate;
  ASSERT_TRUE(snapshot.TryWrite(value));
  std::thread reader([&] { snapshot.Read(); });
  {
    std::unique_lock<std::mutex> lock(gate.mutex);
    gate.cv.wait(lock, [&] { return gate.copying; });
  }
  EXPECT_FALSE(snapshot.TryWrite(value));
  {
    std::lock_guard<std::mutex> lock(gate.mutex);
    gate.release = true;
  }
  gate.cv.notify_all();
  reader.join();
  EXPECT_TRUE(snapshot.TryWrite(value));
}
