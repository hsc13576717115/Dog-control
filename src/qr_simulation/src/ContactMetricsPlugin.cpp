#include <array>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <qr_interfaces/msg/contact_metrics.hpp>

namespace qr_simulation {
// Each foot's metric is the time integral of mean tangential material-point
// relative speed over its current manifold. Unlike foot-centre displacement,
// rolling without sliding contributes zero. Gazebo's discrete contact/velocity
// solution still has numerical error: this is a simulation measurement.
class ContactMetricsPlugin final : public gazebo::WorldPlugin {
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    pub_ = node_->create_publisher<qr_interfaces::msg::ContactMetrics>(
        "/evaluation/contact_metrics", rclcpp::SensorDataQoS());
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    update_ =
        gazebo::event::Events::ConnectWorldUpdateEnd([this] { Update(); });
  }

private:
  static ignition::math::Vector3d
  VelocityAt(const gazebo::physics::LinkPtr &link,
             const ignition::math::Vector3d &point) {
    return link->WorldLinearVel() +
           link->WorldAngularVel().Cross(point - link->WorldPose().Pos());
  }
  void Update() {
    const double now = world_->SimTime().Double();
    const double dt = now - previous_;
    previous_ = now;
    if (dt <= 0 || dt > .01) {
      if (dt < 0) {
        slip_.fill(0);
        duration_.fill(0);
      }
      return;
    }
    std::array<double, 4> speed{};
    std::array<unsigned, 4> count{};
    const std::array<std::string, 4> names{"FR_foot", "FL_foot", "RR_foot",
                                           "RL_foot"};
    auto *manager = world_->Physics()->GetContactManager();
    for (unsigned k = 0; k < manager->GetContactCount(); ++k) {
      const auto *contact = manager->GetContact(k);
      if (!contact || !contact->collision1 || !contact->collision2)
        continue;
      const auto a = contact->collision1->GetLink(),
                 b = contact->collision2->GetLink();
      for (size_t foot = 0; foot < 4; ++foot) {
        const std::string name = "custom_dog::" + names[foot];
        if (a->GetScopedName() != name && b->GetScopedName() != name)
          continue;
        for (int j = 0; j < contact->count; ++j) {
          auto normal = contact->normals[j];
          if (normal.Length() < 1e-9)
            continue;
          normal.Normalize();
          const auto relative = VelocityAt(a, contact->positions[j]) -
                                VelocityAt(b, contact->positions[j]);
          speed[foot] += (relative - normal * relative.Dot(normal)).Length();
          ++count[foot];
        }
      }
    }
    qr_interfaces::msg::ContactMetrics msg;
    msg.header.frame_id = "world";
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
    msg.physics_sequence = ++sequence_;
    for (size_t foot = 0; foot < 4; ++foot) {
      msg.contacts[foot] = count[foot] > 0;
      if (count[foot]) {
        slip_[foot] += dt * speed[foot] / count[foot];
        duration_[foot] += dt;
      }
    }
    msg.cumulative_slip_m = slip_;
    msg.cumulative_contact_s = duration_;
    if (now - published_ >= .01) {
      pub_->publish(msg);
      published_ = now;
    }
  }
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<qr_interfaces::msg::ContactMetrics>::SharedPtr pub_;
  gazebo::event::ConnectionPtr update_;
  std::array<double, 4> slip_{}, duration_{};
  double previous_ = 0, published_ = 0;
  uint64_t sequence_ = 0;
};
GZ_REGISTER_WORLD_PLUGIN(ContactMetricsPlugin)
} // namespace qr_simulation
