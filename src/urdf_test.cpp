#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_test");

  ros::NodeHandle nh;

  const std::string model_param_name("robot_description");
  const auto res = nh.hasParam(model_param_name);
  std::string robot_model_str("");
  if (!res || !nh.getParam(model_param_name, robot_model_str))
  {
    std::cerr << "Robot descripion couldn't be retrieved from param server." << std::endl;
    return EXIT_FAILURE;
  }

  boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));
  if (!model)
  {
    std::cerr << "ERROR: Model Parsing failed" << std::endl;
    return EXIT_FAILURE;
  }

  ros::NodeHandle nh_priv("~");

  std::string wheel_name("steering_joint");
  nh_priv.param("joint", wheel_name, wheel_name);

  std::string base_frame_id("base_link");
  nh_priv.param("base_frame_id", base_frame_id, base_frame_id);

  boost::shared_ptr<const urdf::Joint> wheel_joint(model->getJoint(wheel_name));
  if (!wheel_joint)
  {
    std::cerr << "ERROR: Failed to get joint " << wheel_name << std::endl;
    return EXIT_FAILURE;
  }

  double dx = 0.0;
  double dy = 0.0;
  for (auto joint = wheel_joint; joint && joint->child_link_name != base_frame_id;
       joint = model->getLink(joint->parent_link_name)->parent_joint)
  {
    dx += joint->parent_to_joint_origin_transform.position.x;
    dy += joint->parent_to_joint_origin_transform.position.y;
  }

  const auto d = std::hypot(dx, dy);

  std::cout << "x = " << dx << ", y = " << dy << ", d = " << d << std::endl;

  return EXIT_SUCCESS;
}
