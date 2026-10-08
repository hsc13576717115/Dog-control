#include "custom_dog_rl/OnnxPolicy.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace custom_dog_rl {
namespace {

Ort::SessionOptions makeOptions(int threads) {
  if (threads < 1) {
    throw std::invalid_argument("ONNX Runtime threads must be at least one");
  }
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  // Avoid a background busy-spin competing with the hardware I/O loop.
  options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  options.AddConfigEntry("session.inter_op.allow_spinning", "0");
  return options;
}

void checkTensor(const Ort::Session& session, bool input, const char* name,
                 std::int64_t width) {
  Ort::AllocatorWithDefaultOptions allocator;
  const auto actual_name = input ? session.GetInputNameAllocated(0, allocator)
                                 : session.GetOutputNameAllocated(0, allocator);
  if (std::string(actual_name.get()) != name) {
    throw std::invalid_argument(std::string("Unexpected ONNX tensor name; expected ") + name);
  }
  const auto type_info = input ? session.GetInputTypeInfo(0) : session.GetOutputTypeInfo(0);
  if (type_info.GetONNXType() != ONNX_TYPE_TENSOR) {
    throw std::invalid_argument(std::string("ONNX value must be a tensor: ") + name);
  }
  const auto tensor = type_info.GetTensorTypeAndShapeInfo();
  if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      tensor.GetShape() != std::vector<std::int64_t>({1, width})) {
    throw std::invalid_argument(std::string("ONNX tensor must be FP32 [1,") +
                                std::to_string(width) + "]: " + name);
  }
}

}  // namespace

struct OnnxPolicy::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "custom_dog_rl"};
  Ort::SessionOptions options;
  Ort::Session session;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Observation input_buffer{};
  Action output_buffer{};
  const std::array<std::int64_t, 2> input_shape{1, kObservationSize};
  const std::array<std::int64_t, 2> output_shape{1, kActionSize};
  Ort::Value input_tensor;
  Ort::Value output_tensor;

  Impl(const std::string& path, int threads)
      : options(makeOptions(threads)),
        session(env, path.c_str(), options),
        input_tensor(Ort::Value::CreateTensor<float>(memory, input_buffer.data(),
            input_buffer.size(), input_shape.data(), input_shape.size())),
        output_tensor(Ort::Value::CreateTensor<float>(memory, output_buffer.data(),
            output_buffer.size(), output_shape.data(), output_shape.size())) {
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) {
      throw std::invalid_argument("Combined HIM model requires exactly one input and one output");
    }
    checkTensor(session, true, "obs_history", kObservationSize);
    checkTensor(session, false, "actions", kActionSize);
  }
};

OnnxPolicy::OnnxPolicy(const std::string& path, int threads) {
  if (path.empty()) {
    throw std::invalid_argument("ONNX model path is empty");
  }
  impl_ = std::make_unique<Impl>(path, threads);
}

OnnxPolicy::~OnnxPolicy() = default;

Action OnnxPolicy::infer(const Observation& observation) {
  for (const float value : observation) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Nonfinite ONNX observation");
    }
  }
  std::copy(observation.begin(), observation.end(), impl_->input_buffer.begin());
  const char* input_names[] = {"obs_history"};
  const char* output_names[] = {"actions"};
  impl_->session.Run(Ort::RunOptions{nullptr}, input_names, &impl_->input_tensor, 1,
                     output_names, &impl_->output_tensor, 1);
  for (const float value : impl_->output_buffer) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("ONNX policy produced a nonfinite action");
    }
  }
  return impl_->output_buffer;
}

}  // namespace custom_dog_rl
