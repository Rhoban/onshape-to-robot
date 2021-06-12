#!/usr/bin/env bash

set -e 

pkg_name=$1
echo "

 ▄▄▄▄    ██▓  ▄████     ▄▄▄▄    ▄▄▄       ███▄    █   ▄████ 
▓█████▄ ▓██▒ ██▒ ▀█▒   ▓█████▄ ▒████▄     ██ ▀█   █  ██▒ ▀█▒
▒██▒ ▄██▒██▒▒██░▄▄▄░   ▒██▒ ▄██▒██  ▀█▄  ▓██  ▀█ ██▒▒██░▄▄▄░
▒██░█▀  ░██░░▓█  ██▓   ▒██░█▀  ░██▄▄▄▄██ ▓██▒  ▐▌██▒░▓█  ██▓
░▓█  ▀█▓░██░░▒▓███▀▒   ░▓█  ▀█▓ ▓█   ▓██▒▒██░   ▓██░░▒▓███▀▒
░▒▓███▀▒░▓   ░▒   ▒    ░▒▓███▀▒ ▒▒   ▓▒█░░ ▒░   ▒ ▒  ░▒   ▒ 
▒░▒   ░  ▒ ░  ░   ░    ▒░▒   ░   ▒   ▒▒ ░░ ░░   ░ ▒░  ░   ░ 
 ░    ░  ▒ ░░ ░   ░     ░    ░   ░   ▒      ░   ░ ░ ░ ░   ░ 
 ░       ░        ░     ░            ░  ░         ░       ░ 
      ░                      ░                              

"

echo "Spawning ${pkg_name}_description directory..."

catkin_create_pkg ${pkg_name}_description rospy
cd ${pkg_name}_description/
mkdir launch meshes rviz urdf

cd urdf/
echo
echo "Enter config details : "
echo

read -p 'documentId : ' doc_id
read -p 'robotName : ' robotName
read -p 'outputFormat: [urdf/sdf] : ' outputFormat
read -p 'addDummyBaseLink: [true/false] : ' addDummyBaseLink
read -p 'mergeSTLs: [no/visual/collision/all] : ' mergeSTLs

cat >./config.json <<EOF
    {
        "documentId": "${doc_id}",
        "robotName": "${robotName}",
        "outputFormat": "${outputFormat}",
        "addDummyBaseLink": "${addDummyBaseLink}",
        "mergeSTLs": "${mergeSTLs}",
        "packageName": "${pkg_name}_description/meshes/"
    }
EOF

echo
cd ../
echo

onshape-to-robot urdf

cp urdf/{*.stl,*.part} meshes/
rm -r urdf/{*.stl,*.part}

cd launch/

wget https://gist.githubusercontent.com/KamalanathanN/d62e6a1f2a067a8ee6d79ccdd26444b2/raw/68a637d1b1a363ef9bd5ad46965a09f54ed3a88d/urdf.rviz

cat >./display.launch <<EOF
<launch>
  <arg default="\$(find ${pkg_name}_description)/urdf/robot.urdf" name="model"/>
  <arg default="true" name="gui"/>
  <arg default="\$(find ${pkg_name}_description)/launch/urdf.rviz" name="rvizconfig"/>
  <param command="\$(find xacro)/xacro \$(arg model)" name="robot_description"/>
  <param name="use_gui" value="\$(arg gui)"/>
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <node args="-d \$(arg rvizconfig)" name="rviz" pkg="rviz" required="true" type="rviz"/>
</launch>
EOF

cat >./gazebo.launch <<EOF
<launch>
  <param command="\$(find xacro)/xacro \$(find ${pkg_name}_description)/urdf/robot.urdf" name="robot_description"/>
  <node args="-param robot_description -urdf -model robot" name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"/>
  <include file="\$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
EOF
