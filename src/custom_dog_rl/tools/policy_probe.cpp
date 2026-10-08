#include "custom_dog_rl/OnnxPolicy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Options {
  std::string model;
  std::string input;
  std::string output;
  int threads = 1;
  int warmup = 100;
  int repeat = 1;
};

void usage() {
  std::cout << "policy_probe --model policy_combined.onnx --input observations.f32 "
               "--output actions.f32 [--threads 1] [--warmup 100] [--repeat 1]\n"
               "Input/output: headerless native little-endian float32, N*270 / N*12.\n"
               "Writes one action per input; repeat affects the latency benchmark only.\n";
}

int integer(const std::string& value, bool allow_zero) {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != value.size() || result < (allow_zero ? 0 : 1)) {
    throw std::invalid_argument("Invalid positive integer: " + value);
  }
  return result;
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      usage();
      std::exit(EXIT_SUCCESS);
    }
    if (++i >= argc) throw std::invalid_argument("Missing argument for " + key);
    const std::string value = argv[i];
    if (key == "--model") options.model = value;
    else if (key == "--input") options.input = value;
    else if (key == "--output") options.output = value;
    else if (key == "--threads") options.threads = integer(value, false);
    else if (key == "--warmup") options.warmup = integer(value, true);
    else if (key == "--repeat") options.repeat = integer(value, false);
    else throw std::invalid_argument("Unknown option " + key);
  }
  if (options.model.empty() || options.input.empty() || options.output.empty()) {
    throw std::invalid_argument("--model, --input and --output are required");
  }
  return options;
}

double percentile(const std::vector<double>& sorted, double quantile) {
  return sorted[static_cast<std::size_t>(quantile * (sorted.size() - 1))];
}
}  // namespace

int main(int argc, char** argv) {
  using namespace custom_dog_rl;
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "Probe requires IEEE754 float32");
  try {
    const auto options = parse(argc, argv);
    const std::uint16_t endian_test = 1;
    if (*reinterpret_cast<const unsigned char*>(&endian_test) != 1) {
      throw std::runtime_error("Probe binary interchange requires little-endian host");
    }
    const auto byte_count = std::filesystem::file_size(options.input);
    constexpr auto frame_bytes = kObservationSize * sizeof(float);
    if (byte_count == 0 || byte_count % frame_bytes != 0 || byte_count > 1024ULL * 1024 * 1024) {
      throw std::invalid_argument("Observation file must contain nonempty complete 270D FP32 rows, <=1 GiB");
    }
    if (std::filesystem::exists(options.output)) {
      throw std::invalid_argument("Refusing to overwrite existing probe output: " + options.output);
    }
    std::vector<Observation> observations(byte_count / frame_bytes);
    std::ifstream input(options.input, std::ios::binary);
    if (!input.read(reinterpret_cast<char*>(observations.data()), static_cast<std::streamsize>(byte_count))) {
      throw std::runtime_error("Failed to read observation file");
    }
    for (const auto& observation : observations) {
      for (const float value : observation) {
        if (!std::isfinite(value)) throw std::invalid_argument("Observation file contains NaN or infinity");
      }
    }
    OnnxPolicy policy(options.model, options.threads);
    for (int i = 0; i < options.warmup; ++i) {
      policy.infer(observations[static_cast<std::size_t>(i) % observations.size()]);
    }
    std::vector<Action> actions(observations.size());
    std::vector<double> latency_us;
    latency_us.reserve(observations.size() * static_cast<std::size_t>(options.repeat));
    for (int repeat = 0; repeat < options.repeat; ++repeat) {
      for (std::size_t i = 0; i < observations.size(); ++i) {
        const auto start = std::chrono::steady_clock::now();
        const auto result = policy.infer(observations[i]);
        const auto stop = std::chrono::steady_clock::now();
        latency_us.push_back(std::chrono::duration<double, std::micro>(stop - start).count());
        if (repeat == 0) actions[i] = result;
      }
    }
    std::ofstream output(options.output, std::ios::binary);
    output.write(reinterpret_cast<const char*>(actions.data()),
                 static_cast<std::streamsize>(actions.size() * kActionSize * sizeof(float)));
    output.flush();
    if (!output) throw std::runtime_error("Failed to write action file");
    const double mean = std::accumulate(latency_us.begin(), latency_us.end(), 0.0) / latency_us.size();
    std::sort(latency_us.begin(), latency_us.end());
    std::cout << "{\"samples\":" << observations.size()
              << ",\"threads\":" << options.threads
              << ",\"benchmark_calls\":" << latency_us.size()
              << ",\"latency_us\":{\"mean\":" << mean
              << ",\"p50\":" << percentile(latency_us, 0.50)
              << ",\"p95\":" << percentile(latency_us, 0.95)
              << ",\"p99\":" << percentile(latency_us, 0.99)
              << ",\"max\":" << latency_us.back() << "}}\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "policy_probe: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
