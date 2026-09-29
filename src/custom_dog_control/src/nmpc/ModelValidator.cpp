#include "custom_dog_control/nmpc/ModelValidator.hpp"

#include <cmath>
#include <sstream>

#include <Eigen/Eigenvalues>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

std::string ModelValidationResult::Summary() const {
  std::ostringstream stream;
  stream << (ok ? "model valid" : "model invalid") << ", mass=" << total_mass_kg << " kg";
  for (const auto& error : errors) {
    stream << "; " << error;
  }
  return stream.str();
}

ModelValidationResult ModelValidator::Validate(
    ocs2::PinocchioInterface& interface,
    const ocs2::CentroidalModelInfo& info,
    double expected_mass_kg,
    double mass_tolerance_kg) {
  ModelValidationResult result;
  result.model_joint_slots.fill(-1);
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  // 控制器的固定数组和状态切片依赖这些维数，必须在创建运行时适配器前核对。
  if (info.actuatedDofNum != kJointCount) {
    result.errors.emplace_back("actuated DoF count is not 12");
  }
  if (info.generalizedCoordinatesNum != 6 + kJointCount ||
      info.stateDim != kCentroidalStateDim ||
      info.inputDim != kCentroidalInputDim) {
    result.errors.emplace_back("centroidal state/input dimensions are invalid");
  }
  if (info.numThreeDofContacts != kLegCount) {
    result.errors.emplace_back("3-DoF contact count is not 4");
  }

  for (const auto joint_name : kJointNames) {
    if (!model.existJointName(std::string(joint_name))) {
      result.errors.emplace_back("missing joint " + std::string(joint_name));
    }
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!model.existJointName(std::string(kJointNames[i]))) {
      continue;
    }
    const auto joint_id = model.getJointId(std::string(kJointNames[i]));
    const auto& joint = model.joints[joint_id];
    // 硬件顺序与 URDF 遍历顺序可能不同；保存显式映射，不能直接用关节数组下标。
    const int q_slot = joint.idx_q() - 6;
    const int v_slot = joint.idx_v() - 6;
    if (joint.nq() != 1 || joint.nv() != 1 || q_slot < 0 ||
        q_slot >= static_cast<int>(kJointCount) || v_slot != q_slot) {
      std::ostringstream detail;
      detail << "joint ordering/direction contract failed for "
             << kJointNames[i] << " (nq=" << joint.nq()
             << ", nv=" << joint.nv() << ", idx_q=" << joint.idx_q()
             << ", idx_v=" << joint.idx_v() << ")";
      result.errors.emplace_back(detail.str());
    } else {
      result.model_joint_slots[i] = q_slot;
    }
    if (!(model.lowerPositionLimit(joint.idx_q()) <
          model.upperPositionLimit(joint.idx_q())) ||
        model.effortLimit(joint.idx_v()) <= 0.0 ||
        model.velocityLimit(joint.idx_v()) <= 0.0) {
      result.errors.emplace_back(
          "invalid joint limits for " + std::string(kJointNames[i]));
    }
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (result.model_joint_slots[i] < 0) {
      continue;
    }
    for (std::size_t j = i + 1; j < kJointCount; ++j) {
      if (result.model_joint_slots[i] == result.model_joint_slots[j]) {
        result.errors.emplace_back("joint model slots are not unique");
      }
    }
  }
  for (const auto foot_name : kFootFrameNames) {
    if (!model.existFrame(std::string(foot_name))) {
      result.errors.emplace_back("missing foot frame " + std::string(foot_name));
    }
  }

  // 跳过 universe；检查模型中合并后的刚体惯量，而不是仅统计 URDF 的 link 标签。
  for (std::size_t i = 1; i < model.inertias.size(); ++i) {
    const auto& inertia = model.inertias[i];
    result.total_mass_kg += inertia.mass();
    if (!std::isfinite(inertia.mass()) || inertia.mass() <= 0.0) {
      result.errors.emplace_back("link inertia has non-positive mass");
      continue;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        inertia.inertia().matrix());
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= 0.0) {
      result.errors.emplace_back("link inertia tensor is not positive definite");
    }
  }
  if (std::abs(result.total_mass_kg - expected_mass_kg) > mass_tolerance_kg) {
    result.errors.emplace_back("total mass differs from CAD contract");
  }

  if (model.lowerPositionLimit.size() != model.nq ||
      model.upperPositionLimit.size() != model.nq) {
    result.errors.emplace_back("joint position limits have invalid dimensions");
  } else if (!model.lowerPositionLimit.allFinite() || !model.upperPositionLimit.allFinite()) {
    result.errors.emplace_back("joint position limits contain non-finite values");
  }
  if (model.effortLimit.size() != model.nv || !model.effortLimit.allFinite()) {
    result.errors.emplace_back("joint effort limits are missing or non-finite");
  }

  // 在统一名义姿态下验证几何约定。这里检查的是模型一致性，不能替代真机零位标定。
  try {
    ocs2::vector_t q = pinocchio::neutral(model);
    constexpr std::array<double, kJointsPerLeg> kNominalLeg = {
        0.0, 0.80, -1.60};
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint_id = model.getJointId(std::string(kJointNames[i]));
      q(model.joints[joint_id].idx_q()) = kNominalLeg[i % kJointsPerLeg];
    }
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      const auto foot_name = kFootFrameNames[leg];
      const auto frame_id = model.getFrameId(std::string(foot_name));
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model.nv);
      jacobian.setZero();
      pinocchio::getFrameJacobian(
          model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
      if (!data.oMf[frame_id].translation().allFinite() || !jacobian.allFinite()) {
        result.errors.emplace_back("non-finite FK/Jacobian for " + std::string(foot_name));
      }
      const auto& position = data.oMf[frame_id].translation();
      // REP-103：x 向前、y 向左、z 向上。足端应位于对应象限且低于基座。
      const bool front_expected = leg < 2;
      const bool left_expected = leg == 1 || leg == 3;
      if ((front_expected && position.x() <= 0.0) ||
          (!front_expected && position.x() >= 0.0) ||
          (left_expected && position.y() <= 0.0) ||
          (!left_expected && position.y() >= 0.0) ||
          position.z() >= -0.10) {
        result.errors.emplace_back(
            "foot quadrant/joint direction check failed for " +
            std::string(foot_name));
      }
    }
  } catch (const std::exception& exception) {
    result.errors.emplace_back(std::string("Pinocchio FK/Jacobian failed: ") + exception.what());
  }

  result.ok = result.errors.empty();
  return result;
}

}  // namespace custom_dog_control
