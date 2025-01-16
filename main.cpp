// Copyright 2021 RT Corporation
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

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>

#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"



void set_arm_joint_positions(std::vector<manipulators_link::Link> & links,
                             std::vector<double> positions) {
  // リンクにarmジョイントの現在角度をセットする
  if (positions.size() != 7) {
    std::cerr << "引数positionsには7個のジョイント角度をセットしてください" << std::endl;
    return;
  }

  int start_id = 2;  // Link1
  for (int i=0; i < positions.size(); i++) {
    links[start_id + i].q = positions[i];
  }
}

void move_to(rt_manipulators_cpp::Hardware & hardware,
             const kinematics_utils::links_t & links,
             Eigen::Vector3d & target_p, const Eigen::Matrix3d & target_R) {
  // 目標位置・姿勢をもとにIKを解き、関節角度をセットする
  std::cout << "目標位置:" << std::endl << target_p << std::endl;
  std::cout << "目標姿勢:" << std::endl << target_R << std::endl;
  std::cout << "----------------------" << std::endl;
  kinematics_utils::q_list_t q_list;
  if (kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list) == false) {
    std::cout << "IKに失敗しました" << std::endl;
  } else {
    std::cout << "IKに成功しました" << std::endl;
  }
  for (const auto & [target_id, q_value] : q_list) {
    hardware.set_position(links[target_id].dxl_id, q_value);
  }
}

int main() {


  std::cout << "手先目標位置・姿勢をもとに逆運動学を解き、"
            << "CRANE-X7を動かすサンプルです." << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;  // 3Mbps
  std::string hardware_config_file = "../config/crane-x7.yaml";
  std::string link_config_file = "../config/crane-x7_links.csv";

  auto links = kinematics_utils::parse_link_config_file(link_config_file);
  kinematics::forward_kinematics(links, 1);
  kinematics_utils::print_links(links, 1);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  // 関節可動範囲の設定
  for (auto link_id : kinematics_utils::find_route(links, 8)) {
    hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
    hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
  }

  ////////////arm

  std::cout << "armグループのサーボ最大加速度をpi rad/s^2、最大速度をpi rad/sに設定します."
            << std::endl;
  if (!hardware.write_max_acceleration_to_group("arm", 2.0 * M_PI)) {
    std::cerr << "armグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_max_velocity_to_group("arm", 2.0 * M_PI)) {
    std::cerr << "armグループの最大速度を設定できませんでした." << std::endl;
    return -1;
  }

  std::cout << "armグループのサーボ位置制御PIDゲインに(800, 0, 0)を書き込みます."
            << std::endl;
  if (!hardware.write_position_pid_gain_to_group("arm", 800, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("arm")) {
    std::cerr << "armグループのトルクをONできませんでした." << std::endl;
    return -1;
  }


  ///////hand

  std::cout << "handグループのサーボ最大加速度を0.5pi rad/s^2、最大速度を0.5pi rad/sに設定します."
            << std::endl;
  if (!hardware.write_max_acceleration_to_group("hand", 2.0 * M_PI)) {
    std::cerr << "handグループの最大加速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_max_velocity_to_group("hand", 2.0 * M_PI)) {
    std::cerr << "handグループの最大速度を設定できませんでした." << std::endl;
    return -1;
  }

  if (!hardware.write_position_pid_gain_to_group("hand", 800, 0, 0)) {
    std::cerr << "handグループにPIDゲインを書き込めませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("hand")) {
    std::cerr << "handグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"arm", "hand"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後にX7が垂直姿勢へ移行するため、X7の周りに物や人を近づけないで下さい."
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 垂直姿勢へ移動
  std::vector<double> target_positions(7, 0.0);
  hardware.set_positions("arm", target_positions);
  std::this_thread::sleep_for(std::chrono::seconds(5));


  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  std::cout << "move to set position" << std::endl;
  target_p << 0.2, 0.0, 0.11;
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI, 0);
  move_to(hardware, links, target_p, target_R);
  //hand open
  hardware.set_position("joint_hand", 0.2 * (M_PI));
  std::this_thread::sleep_for(std::chrono::seconds(5));

  hardware.set_position("joint_hand", 0.05);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  double target_x = 0.2;
  double target_y = 0.0;
  double target_z = 0.13;

  double position_x;
  double position_y;

  double step = 0.01;

  bool increasing = true;

  target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI, 0);
  
  while(1){
    std::vector<double> positions;
    if (hardware.get_positions("arm", positions)) {
      set_arm_joint_positions(links, positions);
      kinematics::forward_kinematics(links, 1);
      int target_link = 8;
      auto pos_xyz = links[target_link].p;
      position_x = pos_xyz[0];
      position_y = pos_xyz[1];

    }    
    // auto [cx, cy, vx, vy] = tracker.track_when_it_called();

    // std::vector<float> states={position_x, position_y, cx, cy, vx, vy};

    // auto states_tensor = torch::from_blob(states.data(), {1,6});

    // at::Tensor output = model.forward({states_tensor}).toTensor();
    // float target_x = output[0][0].item<float>();
    // float target_y = output[0][1].item<float>();

    float target_x;
    float target_y;

    std::cin >> target_x >> target_y;

    target_p << target_x, target_y, target_z;
    move_to(hardware, links, target_p, target_R);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (int i = 0; i < 100; ++i){
    if (increasing){
      target_y += step;
      if (target_y > 0.3){
        increasing = false;
        target_y = 0.3;
      }
    } else {
      target_y -= step;
      if (target_y < -0.3){
        increasing = true;
        target_y = -0.3;
      }
    }

    
    target_p << target_x, target_y, target_z;
    move_to(hardware, links, target_p, target_R);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "正面へ移動し、手先を真下へ向ける" << std::endl;
  target_p << 0.2, 0.0, 0.1;
  target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI, 0);
  move_to(hardware, links, target_p, target_R);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  std::cout << "armグループのサーボ位置制御PIDゲインに(5, 0, 0)を書き込み、脱力させます."
            << std::endl;
  if (!hardware.write_position_pid_gain_to_group("arm", 5, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  if (!hardware.write_position_pid_gain_to_group("hand", 5, 0, 0)) {
    std::cerr << "handグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  std::cout << "10秒間スリープします." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  if (!hardware.torque_off("arm")) {
    std::cerr << "armグループのトルクをOFFできませんでした." << std::endl;
  }

  if (!hardware.torque_off("hand")) {
    std::cerr << "handグループのトルクをOFFできませんでした." << std::endl;
  }


  if (!hardware.write_position_pid_gain_to_group("arm", 800, 0, 0)) {
    std::cerr << "armグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  if (!hardware.write_position_pid_gain_to_group("hand", 800, 0, 0)) {
    std::cerr << "handグループにPIDゲインを書き込めませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
