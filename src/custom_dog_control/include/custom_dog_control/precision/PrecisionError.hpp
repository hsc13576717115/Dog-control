#pragma once
#include <cstdint>
namespace custom_dog_control {
enum class PrecisionError : uint16_t {
#define QR_ERROR(name, value, text) name = value,
#include "custom_dog_control/precision/PrecisionError.def"
#undef QR_ERROR
};
inline const char *ErrorName(PrecisionError error) {
  switch (error) {
#define QR_ERROR(name, value, text)                                            \
  case PrecisionError::name:                                                   \
    return text;
#include "custom_dog_control/precision/PrecisionError.def"
#undef QR_ERROR
  }
  return "unknown_error";
}
} // namespace custom_dog_control
