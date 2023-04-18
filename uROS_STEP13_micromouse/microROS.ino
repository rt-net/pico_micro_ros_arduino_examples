// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

tf2_msgs__msg__TFMessage * tf_message;
sensor_msgs__msg__JointState jstate;
std_msgs__msg__Int16 bat_msg;
rosidl_runtime_c__String joint_name[2];
double positions[2];
pico_msgs__msg__LightSensor sensor_msg;
visualization_msgs__msg__Marker marker_msg;

rcl_publisher_t publisher_tf, publisher_joint, publisher_sensor, publisher_battery,
  publisher_marker;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

extern unsigned char second_run[256];

void marker_set(int start2_x, int start2_y, t_direction_glob dir)
{
  int i = 0;
  t_direction_glob temp_dir = dir;
  int x, y;

  x = start2_x, y = start2_y;

  switch (temp_dir) {
    case north:
      y++;
      break;
    case east:
      x++;
      break;
    case south:
      y--;
      break;
    case west:
      x--;
      break;
  }

  while (1) {
    //直線
    if (second_run[i] != 0) {
      if (second_run[i] == 127) {
        return;
      }
      for (int j = 0; j < second_run[i]; j++) {
        switch (temp_dir) {
          case north:
            fast_path_add_publish(y, x, north);
            y++;
            break;
          case east:
            fast_path_add_publish(y, x, east);
            x++;
            break;
          case south:
            fast_path_add_publish(y, x, south);
            y--;
            break;
          case west:
            fast_path_add_publish(y, x, west);
            x--;
            break;
        }
        delay(30);
      }
    }
    i++;
    //コーナー
    if (second_run[i] == 127) break;
    switch (second_run[i]) {
      case R90:
        if (temp_dir == north)
          temp_dir = east;
        else if (temp_dir == east)
          temp_dir = south;
        else if (temp_dir == south)
          temp_dir = west;
        else
          temp_dir = north;
        break;
      case L90:
        if (temp_dir == north)
          temp_dir = west;
        else if (temp_dir == east)
          temp_dir = north;
        else if (temp_dir == south)
          temp_dir = east;
        else
          temp_dir = south;
        break;
    }
    switch (temp_dir) {
      case north:
        fast_path_add_publish(y, x, north);
        y++;
        break;
      case east:
        fast_path_add_publish(y, x, east);
        x++;
        break;
      case south:
        fast_path_add_publish(y, x, south);
        y--;
        break;
      case west:
        fast_path_add_publish(y, x, west);
        x--;
        break;
    }
    i++;
    delay(30);
  }
  fast_task = false;
  delay(10);
}

//micro-ROS function
void error_loop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

const void euler_to_quat(float x, float y, float z, double * q)
{
  float c1 = cos(y / 2);
  float c2 = cos(z / 2);
  float c3 = cos(x / 2);

  float s1 = sin(y / 2);
  float s2 = sin(z / 2);
  float s3 = sin(x / 2);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

void pole_publish(int x, int y)
{  //x,yの方向はrvizの座標
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 1;  //cube=1
  marker_msg.ns.data = "pole";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 0;  //0=add 2=del 3=del_all
  marker_msg.id = y * 2 + x * 2 * 256;
  marker_msg.scale.x = 0.012;
  marker_msg.scale.y = 0.012;
  marker_msg.scale.z = 0.049;
  marker_msg.color.r = 1.0f;
  marker_msg.color.g = 1.0f;
  marker_msg.color.b = 1.0f;
  marker_msg.color.a = 1.0f;
  marker_msg.pose.position.x = -0.090 + x * 0.180;
  marker_msg.pose.position.y = 0.090 - y * 0.180;
  marker_msg.pose.position.z = 0.0245;
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
  delay(10);

  //柱のトップの赤いところ
  marker_msg.id = y * 2 + x * 2 * 256 + 0x4000;
  marker_msg.scale.x = 0.012;
  marker_msg.scale.y = 0.012;
  marker_msg.scale.z = 0.001;
  marker_msg.color.r = 1.0f;
  marker_msg.color.g = 0.0f;
  marker_msg.color.b = 0.0f;
  marker_msg.color.a = 1.0f;

  marker_msg.pose.position.x = -0.090 + x * 0.180;
  marker_msg.pose.position.y = 0.090 - y * 0.180;
  marker_msg.pose.position.z = 0.0495;
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
}

void wall_publish(int x, int y, t_direction_glob dir)
{  //x,yの方向はrvizの座標、t_direction_globはマイクマウスのマップの方角
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 1;  //cube=1
  marker_msg.ns.data = "wall";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 0;  //0=add 2=del 3=del_all
  marker_msg.color.r = 1.0f;
  marker_msg.color.g = 1.0f;
  marker_msg.color.b = 1.0f;
  marker_msg.color.a = 1.0f;
  switch (dir) {
    case north:
      marker_msg.id = (y * 2 + 1) + (x + 1) * 2 * 256;
      marker_msg.scale.x = 0.012;
      marker_msg.scale.y = 0.168;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = 0.090 + x * 0.180;
      marker_msg.pose.position.y = -y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case south:
      marker_msg.id = (y * 2 + 1) + x * 2 * 256;
      marker_msg.scale.x = 0.012;
      marker_msg.scale.y = 0.168;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = -0.090 + x * 0.180;
      marker_msg.pose.position.y = -y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case east:
      marker_msg.id = (y + 1) * 2 + (x * 2 + 1) * 256;
      marker_msg.scale.x = 0.168;
      marker_msg.scale.y = 0.012;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = x * 0.180;
      marker_msg.pose.position.y = -0.090 - y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case west:
      marker_msg.id = y * 2 + (x * 2 + 1) * 256;
      marker_msg.scale.x = 0.168;
      marker_msg.scale.y = 0.012;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = x * 0.180;
      marker_msg.pose.position.y = 0.090 - y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
  }
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
}

void vwall_publish(int x, int y, t_direction_glob dir)
{  //x,yの方向はrvizの座標、t_direction_globはマイクマウスのマップの方角
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 1;  //cube=1
  marker_msg.ns.data = "wall";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 0;  //0=add 2=del 3=del_all
  marker_msg.color.r = 0.5f;
  marker_msg.color.g = 0.5f;
  marker_msg.color.b = 1.0f;
  marker_msg.color.a = 0.2f;
  switch (dir) {
    case north:
      marker_msg.id = (y * 2 + 1) + (x + 1) * 2 * 256;
      marker_msg.scale.x = 0.012;
      marker_msg.scale.y = 0.168;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = 0.090 + x * 0.180;
      marker_msg.pose.position.y = -y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case south:
      marker_msg.id = (y * 2 + 1) + x * 2 * 256;
      marker_msg.scale.x = 0.012;
      marker_msg.scale.y = 0.168;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = -0.090 + x * 0.180;
      marker_msg.pose.position.y = -y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case east:
      marker_msg.id = (y + 1) * 2 + (x * 2 + 1) * 256;
      marker_msg.scale.x = 0.168;
      marker_msg.scale.y = 0.012;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = x * 0.180;
      marker_msg.pose.position.y = -0.090 - y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
    case west:
      marker_msg.id = y * 2 + (x * 2 + 1) * 256;
      marker_msg.scale.x = 0.168;
      marker_msg.scale.y = 0.012;
      marker_msg.scale.z = 0.050;
      marker_msg.pose.position.x = x * 0.180;
      marker_msg.pose.position.y = 0.090 - y * 0.180;
      marker_msg.pose.position.z = 0.025;
      break;
  }
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
}

void wall_del_publish(int x, int y, t_direction_glob dir)
{
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 1;  //cube=1
  marker_msg.ns.data = "wall";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 2;  //0=add 2=del 3=del_all
  switch (dir) {
    case north:
      marker_msg.id = (y * 2 + 1) + (x + 1) * 2 * 256;
      break;
    case south:
      marker_msg.id = (y * 2 + 1) + x * 2 * 256;
      break;
    case east:
      marker_msg.id = (y + 1) * 2 + (x * 2 + 1) * 256;
      break;
    case west:
      marker_msg.id = y * 2 + (x * 2 + 1) * 256;
      break;
  }
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
}

void fast_path_del_publish(void)
{
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 0;  //0:arrow
  marker_msg.ns.data = "fast";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 3;  //0=add 2=del 3=del_all
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
}

void fast_path_add_publish(int x, int y, t_direction_glob dir)
{
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.type = 0;  //0:arrow
  marker_msg.ns.data = "fast";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.action = 0;  //0=add 2=del 3=del_all
  marker_msg.color.r = 0.0f;
  marker_msg.color.g = 1.0f;
  marker_msg.color.b = 0.0f;
  marker_msg.color.a = 1.0f;

  marker_msg.scale.x = 0.100;
  marker_msg.scale.y = 0.030;
  marker_msg.scale.z = 0.030;

  marker_msg.pose.position.x = x * 0.180;
  marker_msg.pose.position.y = -y * 0.180;
  marker_msg.pose.position.z = 0.0;

  marker_msg.id = y + x * 256;

  switch (dir) {
    case north:
      break;
    case south:
      marker_msg.pose.orientation.z = 1;  //矢印を下にするためz軸を回転
      marker_msg.pose.orientation.w = 0;  //矢印を下にするためz軸を回転

      break;
    case west:
      marker_msg.pose.orientation.z = 0.707;  //矢印を下にするためz軸を回転
      marker_msg.pose.orientation.w = 0.707;  //矢印を下にするためz軸を回転
      break;
    case east:
      marker_msg.pose.orientation.z = -0.707;  //矢印を下にするためz軸を回転
      marker_msg.pose.orientation.w = 0.707;   //矢印を下にするためz軸を回転
      break;
  }
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
  marker_msg.pose.orientation.z = 0.0;  //矢印を下にするためz軸を回転
  marker_msg.pose.orientation.w = 1.0;  //矢印を下にするためz軸を回転
}

//mico-ROSのタスク
void publisher_task(void * pvParameters)
{
  double q[4];
  uint32_t current;

  while (1) {
    current = micros();

    jstate.header.stamp.sec = current / 1000000;
    jstate.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;

    euler_to_quat(0, 0, odom_theta, q);
    tf_message->transforms.data[0].transform.translation.x = odom_x;
    tf_message->transforms.data[0].transform.translation.y = odom_y;
    tf_message->transforms.data[0].transform.translation.z = 0.0;

    tf_message->transforms.data[0].transform.rotation.x = (double)q[1];
    tf_message->transforms.data[0].transform.rotation.y = (double)q[2];
    tf_message->transforms.data[0].transform.rotation.z = (double)q[3];
    tf_message->transforms.data[0].transform.rotation.w = (double)q[0];
    tf_message->transforms.data[0].header.stamp.nanosec = jstate.header.stamp.nanosec;
    tf_message->transforms.data[0].header.stamp.sec = jstate.header.stamp.sec;

    jstate.position.data[0] = position_l;
    jstate.position.data[1] = position_r;

    sensor_msg.forward_r = sen_fr.value;
    sensor_msg.forward_l = sen_fl.value;
    sensor_msg.right = sen_r.value;
    sensor_msg.left = sen_l.value;
    bat_msg.data = battery_value;

    RCSOFTCHECK(rcl_publish(&publisher_tf, tf_message, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_joint, &jstate, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_sensor, &sensor_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_battery, &bat_msg, NULL));

    switch (map_control.get_wall_data(publish_x, publish_y, north)) {
      case WALL:
        wall_publish(publish_y, publish_x, north);
        break;
      case NOWALL:
        wall_del_publish(publish_y, publish_x, north);
        break;
    }

    switch (map_control.get_wall_data(publish_x, publish_y, east)) {
      case WALL:
        wall_publish(publish_y, publish_x, east);
        break;
      case NOWALL:
        wall_del_publish(publish_y, publish_x, east);
        break;
    }

    switch (map_control.get_wall_data(publish_x, publish_y, south)) {
      case WALL:
        wall_publish(publish_y, publish_x, south);
        break;
      case NOWALL:
        wall_del_publish(publish_y, publish_x, south);
        break;
    }

    switch (map_control.get_wall_data(publish_x, publish_y, west)) {
      case WALL:
        wall_publish(publish_y, publish_x, west);
        break;
      case NOWALL:
        wall_del_publish(publish_y, publish_x, west);
        break;
    }
    if (fast_task == true) {  //最短走行時のマーカーを出力
      fast_path_del_publish();
      delay(10);
      marker_set(start_x, start_y, start_dir);
    }
    delay(10);
  }
}

void init_microROS(void)
{
  char a = 0;

  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);

  SetLED(1);
  delay(1000);
  SetLED(3);
  delay(1000);
  SetLED(7);
  delay(1000);
  SetLED(15);
  delay(1000);

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_pico_node", "", &support));
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_tf, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_joint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_sensor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(pico_msgs, msg, LightSensor),
    "/pico_sensor"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_battery, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/pico_battery"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_marker, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
    "/marker"));

  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = "/odom";
  tf_message->transforms.data[0].header.frame_id.size =
    strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  tf_message->transforms.data[0].child_frame_id.data = "/base_footprint";
  tf_message->transforms.data[0].child_frame_id.size =
    strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;

  joint_name[0].data = "left_wheel_joint";
  joint_name[0].size = strlen(joint_name[0].data);
  joint_name[0].capacity = joint_name[0].size + 1;

  joint_name[1].data = "right_wheel_joint";
  joint_name[1].size = strlen(joint_name[1].data);
  joint_name[1].capacity = joint_name[1].size + 1;

  jstate.name.data = joint_name;
  jstate.name.size = 2;
  jstate.name.capacity = 2;
  jstate.position.data = positions;
  jstate.position.size = 2;
  jstate.position.capacity = 2;

  //marker
  marker_msg.header.frame_id.data = "odom";
  marker_msg.header.frame_id.size = strlen(marker_msg.header.frame_id.data);
  marker_msg.header.frame_id.capacity = marker_msg.header.frame_id.size + 1;
  marker_msg.pose.orientation.x = 0.0;
  marker_msg.pose.orientation.y = 0.0;
  marker_msg.pose.orientation.z = 0.0;
  marker_msg.pose.orientation.w = 1.0;

  //goal maker
  uint32_t current = micros();
  marker_msg.header.stamp.sec = current / 1000000;
  marker_msg.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;
  marker_msg.ns.data = "goal";
  marker_msg.ns.size = strlen(marker_msg.ns.data);
  marker_msg.ns.capacity = marker_msg.ns.size + 1;
  marker_msg.type = 0;  //arrow=0
  marker_msg.id = 0x0000;
  marker_msg.scale.x = 0.100;
  marker_msg.scale.y = 0.030;
  marker_msg.scale.z = 0.030;
  marker_msg.color.r = 1.0f;
  marker_msg.color.g = 1.0f;
  marker_msg.color.b = 0.0f;
  marker_msg.color.a = 1.0f;
  marker_msg.pose.position.x = map_control.get_goal_y() * 0.180;
  marker_msg.pose.position.y = -1 * map_control.get_goal_x() * 0.180;
  marker_msg.pose.position.z = 0.050;
  marker_msg.pose.orientation.y = 1.0;  //矢印を下にするためy軸を回転
  RCSOFTCHECK(rcl_publish(&publisher_marker, &marker_msg, NULL));
  delay(10);

  marker_msg.pose.orientation.y = 0.0;  //向きを戻す

  for (int i = 0; i < 17; i++) {  //rviz x axis
    SetLED(a++);
    for (int j = 0; j < 17; j++) {  //rviz y axis
      pole_publish(i, j);
      delay(15);

      if ((i == 0) && (j < 16)) {  //南
        wall_publish(i, j, south);
        delay(15);
      }

      if ((j == 0) && (i < 16)) {  //西
        wall_publish(i, j, west);
        delay(15);
      }
      if ((i == 15) && (j < 16)) {  //北
        wall_publish(i, j, north);
        delay(15);
      }
      if ((i < 15) && (j < 16)) {
        vwall_publish(i, j, north);
        delay(15);
      }

      if ((j == 15) && (i < 16)) {  //東
        wall_publish(i, j, east);
        delay(15);
      }

      if ((i < 15) && (j < 15)) {
        vwall_publish(i, j, east);
        delay(15);
      }
    }
  }
  for (int j = 0; j < 15; j++) {
    vwall_publish(15, j, east);
    delay(15);
  }

  wall_publish(0, 0, east);
  delay(20);
  wall_del_publish(0, 0, north);
  delay(20);

  fast_path_del_publish();
  delay(20);

  xTaskCreateUniversal(
    //  xTaskCreatePinnedToCore(
    publisher_task, "publisher_task", 4096,
    //    8192,  //4096,
    NULL, 1, NULL,
    //    PRO_CPU_NUM
    //    APP_CPU_NUM
    CONFIG_ARDUINO_RUNNING_CORE);
}
