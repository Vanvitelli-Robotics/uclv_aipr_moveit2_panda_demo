#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <std_srvs/srv/set_bool.hpp>

namespace uclv
{
class SceneBuilderNode : public rclcpp::Node
{
private:
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_builder_;

  rclcpp::TimerBase::SharedPtr start_timer_;

public:
  SceneBuilderNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
    : rclcpp::Node("scene_builder", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true))
  {
    start_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SceneBuilderNode::start, this));
  }

  void start()
  {
    start_timer_->cancel();

    static const std::string PLANNING_GROUP = "panda_arm";
    move_group_interface_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), PLANNING_GROUP);

    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

    using std::placeholders::_1;
    using std::placeholders::_2;
    service_builder_ = this->create_service<std_srvs::srv::SetBool>(
        "build_scene", std::bind(&SceneBuilderNode::build_scene_srv_cb, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Ready to add build the scene.");
  }

  ~SceneBuilderNode() = default;

  void addBOX(double dim_x, double dim_y, double dim_z, geometry_msgs::msg::PoseStamped pose, const std::string& obj_id,
              double delta_z_primitive = 0.0)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "#####\nAdding box [" << dim_x << ", " << dim_y << ", " << dim_z
                                                                 << "]\n pose:\n"
                                                                 << geometry_msgs::msg::to_yaml(pose) << "\n#####");

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = pose.header.frame_id;
    collision_object.id = obj_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dim_x;
    primitive.dimensions[primitive.BOX_Y] = dim_y;
    primitive.dimensions[primitive.BOX_Z] = dim_z;

    pose.pose.position.z += dim_z / 2.0 + delta_z_primitive;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose.pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObject(collision_object);

    this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
  }

  void clearScene()
  {
    // RIMUOVO TUTTO GLI OGGETTI PRESENTI
    for (auto const& element : planning_scene_interface_->getAttachedObjects())
    {
      move_group_interface_->detachObject(element.first);
      this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0));
    }
    auto objects_map = planning_scene_interface_->getObjects();
    std::vector<std::string> obj_keys;
    for (auto const& element : objects_map)
    {
      obj_keys.push_back(element.first);
    }
    planning_scene_interface_->removeCollisionObjects(obj_keys);
    this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0));
  }

  void buildScene(bool create_object_to_grasp = true)
  {
    clearScene();

    // COSTRUISCO LA SCENA

    double x1 = 0.5;
    double y1 = 0.60;
    double DX_0 = 0.2;
    // double Dy_f = 0.25;
    double Table_DX = 0.5;
    double Table_DY = 1.0;
    double Table_DZ = 0.4;
    double DX = 0.5;
    double DX_2 = 0.25;
    double DZ_2 = 0.5;
    double DZ_3 = 0.25;

    // box 1
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = x1 + Table_DX / 2.0;
      addBOX(Table_DX, Table_DY, Table_DZ, pose, "obst_1");
    }

    // box 2
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.y = y1 + Table_DX / 2.0;
      addBOX(Table_DY, Table_DX, Table_DZ, pose, "obst_2");
    }

    // box 3
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = DX / 2.0 + DX_2 / 2.0;
      pose.pose.position.y = y1 + Table_DX / 2.0;
      pose.pose.position.z = Table_DZ;
      addBOX(DX_2, Table_DX, DZ_2, pose, "obst_3");
    }

    // box 4
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = -DX / 2.0 - DX_2 / 2.0;
      pose.pose.position.y = y1 + Table_DX / 2.0;
      pose.pose.position.z = Table_DZ;
      addBOX(DX_2, Table_DX, DZ_2, pose, "obst_4");
    }

    // box 5
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = 0.0;
      pose.pose.position.y = y1 + Table_DX / 2.0;
      pose.pose.position.z = Table_DZ + DZ_2;
      addBOX(Table_DY, Table_DX, DZ_3, pose, "obst_5");
    }

    // Object to attach
    if (create_object_to_grasp)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose.pose.position.x = x1 + DX_0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = Table_DZ;
      addBOX(0.02, 0.02, 0.22, pose, "attach_obj");
    }
  }

  void build_scene_srv_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    buildScene(request->data);
    response->message = "OK";
    response->success = true;
  }
};

}  // namespace uclv

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<uclv::SceneBuilderNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}