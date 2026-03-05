#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

struct Obj
{
  std::string name;
  std::string color;   // e.g. "green"
  std::string shape;   // e.g. "cube"
  double x{}, y{}, z{};
};

static std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
    out.push_back(item);
  return out;
}

class FakeVLAStub : public rclcpp::Node
{
public:
  FakeVLAStub() : Node("fake_vla_stub")
  {
    declare_parameter<std::string>("goal_frame", "world");
    declare_parameter<double>("hover_z", 0.12);
    declare_parameter<std::vector<std::string>>("objects", std::vector<std::string>{});

    goal_frame_ = get_parameter("goal_frame").as_string();
    hover_z_ = get_parameter("hover_z").as_double();
    auto obj_lines = get_parameter("objects").as_string_array();

    for (const auto &line : obj_lines)
    {
      auto parts = split(line, ',');
      if (parts.size() != 6)
      {
        RCLCPP_WARN(get_logger(), "Skipping malformed object entry: '%s'", line.c_str());
        continue;
      }
      Obj o;
      o.name = parts[0];
      o.color = to_lower(parts[1]);
      o.shape = to_lower(parts[2]);
      o.x = std::stod(parts[3]);
      o.y = std::stod(parts[4]);
      o.z = std::stod(parts[5]);
      objects_.push_back(o);
    }

    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vla_goal_pose", 10);
    sub_instr_ = create_subscription<std_msgs::msg::String>(
        "/instruction", 10,
        std::bind(&FakeVLAStub::on_instruction, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FakeVLAStub ready. Loaded %zu objects. goal_frame='%s'",
                objects_.size(), goal_frame_.c_str());
  }

private:
  struct Rule
  {
    std::string forbid_color;
    std::string forbid_shape;
  };

  std::optional<std::string> extract_color(const std::string &text) const
  {
    static const std::vector<std::string> colors = {"red", "green", "blue", "yellow"};
    for (const auto &c : colors)
    {
      if (text.find(c) != std::string::npos)
        return c;
    }
    return std::nullopt;
  }

  std::optional<Rule> extract_forbid_rule(const std::string &text) const
  {
    // Expect rule like: "do not pick up the green cube"
    const std::string key = "do not pick up";
    auto pos = text.find(key);
    if (pos == std::string::npos)
      return std::nullopt;

    // Grab the substring after the key and try to find "<color> <shape>"
    auto tail = text.substr(pos + key.size());
    tail = to_lower(tail);

    auto c = extract_color(tail);
    if (!c)
      return std::nullopt;

    static const std::vector<std::string> shapes = {"cube", "cylinder", "pyramid"};
    std::optional<std::string> s;
    for (const auto &sh : shapes)
    {
      if (tail.find(sh) != std::string::npos)
      {
        s = sh;
        break;
      }
    }
    if (!s)
      return std::nullopt;

    return Rule{*c, *s};
  }

  void on_instruction(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string raw = msg->data;
    const std::string text = to_lower(raw);

    auto target_color = extract_color(text);
    if (!target_color)
    {
      RCLCPP_WARN(get_logger(), "No target color found in instruction: '%s'", raw.c_str());
      return;
    }

    auto forbid = extract_forbid_rule(text);

    // Choose first object matching target_color that isn't forbidden
    const Obj *chosen = nullptr;
    for (const auto &o : objects_)
    {
      if (o.color != *target_color)
        continue;

      if (forbid && o.color == forbid->forbid_color && o.shape == forbid->forbid_shape)
        continue;

      chosen = &o;
      break;
    }

    if (!chosen)
    {
      RCLCPP_WARN(get_logger(),
                  "No valid object found for target_color='%s' given rule. Instruction: '%s'",
                  target_color->c_str(), raw.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = goal_frame_;
    goal.pose.position.x = chosen->x;
    goal.pose.position.y = chosen->y;
    goal.pose.position.z = chosen->z + hover_z_;
    goal.pose.orientation.w = 1.0; // simple neutral orientation

    pub_goal_->publish(goal);

    if (forbid)
    {
      RCLCPP_INFO(get_logger(),
                  "Instruction='%s' | target='%s' | forbid='%s %s' | chosen='%s' (%s %s)",
                  raw.c_str(),
                  target_color->c_str(),
                  forbid->forbid_color.c_str(), forbid->forbid_shape.c_str(),
                  chosen->name.c_str(), chosen->color.c_str(), chosen->shape.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Instruction='%s' | target='%s' | chosen='%s' (%s %s)",
                  raw.c_str(), target_color->c_str(),
                  chosen->name.c_str(), chosen->color.c_str(), chosen->shape.c_str());
    }
  }

  std::string goal_frame_;
  double hover_z_{0.12};
  std::vector<Obj> objects_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_instr_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeVLAStub>());
  rclcpp::shutdown();
  return 0;
}