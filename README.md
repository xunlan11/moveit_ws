## 本仓库仅供课程学习使用
## 机器人学导论实验
参考张涛同学的教程：https://github.com/Suixin04/ws_moveit.git
### 系统准备：ubuntu20.04 + ros1 noetic
- ubuntu20.04：略
- ros1 noetic（鱼香一键安装）
  ```wget http://fishros.com/install -O fishros && . fishros```
- 安装依赖
  ```sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential```
- rosdep
  ```sudo apt install python3-rosdep```
  ```sudo rosdep init```
  ```rosdep update```
### 安装movelt并创建工作空间
- 更新：
  ```sudo apt update```
  ```sudo apt dist-upgrade```
- 安装构建工具
  ```sudo apt install ros-noetic-catkin python3-catkin-tools```
  ```sudo apt install python3-wstool```
- 安装movelt及其组件  
  ```sudo apt install ros-noetic-moveit```
  ```sudo apt install ros-noetic-moveit-ros-visualization ros-noetic-moveit-planners ros-noetic-moveit-ros-move-group ros-noetic-moveit-ros-perception```
- 创建工作空间moveit_ws并添加环境变量
  ```mkdir -p ~/moveit_ws/src```
  ```cd ~/moveit_ws/src```
  ```catkin_init_workspace```
  ```cd ~/moveit_ws```
  ```catkin_make```
  ```echo "source ~/moveit_ws/devel/setup.bash" >> ~/.bashrc```
  ```source ~/.bashrc```
### 安装机器臂（iiwa7）包 
- 下载urdf文件
  ```cd ~/moveit_ws/src```
  ```git clone https://github.com/facebookresearch/differentiable-robot-model.git```
  ```sudo apt install ros-noetic-franka-description```
- 导出机械臂包
  ```roslaunch moveit_setup_assistant setup_assistant.launch```
  导出时出现以下错误：OGRE图形渲染引擎和ROS MoveIt Setup Assistant启动失败
  ![error1](./figure/error1.png)
  - 安装mesa工具```sudo ubuntu-drivers autoinstall```
  - 修改urdf文件（位于/home/robot/moveit_ws/src/differentiable-robot-model/diff_robot_data/kuka_iiwa/urdf/iiwa7.urdf）：为<mesh filename="meshes/iiwa7/x/x"/>在meshes前添加../
  - 添加package.xml和CMakeLists.txt，编译使differentiable-robot-model成为ros包
- MoveIt Setup Assistant：配置自碰撞、虚拟基座关节、规划组、原姿态和末端执行器，详见参考内容
- 编译测试
  ```cd ~/moveit_ws```
  ```catkin_make```
  ```roslaunch iiwa7_moveit_config demo.launch```