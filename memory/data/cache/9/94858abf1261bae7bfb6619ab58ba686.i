a:23:{i:0;a:3:{i:0;s:14:"document_start";i:1;a:0:{}i:2;i:0;}i:1;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:0;}i:2;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:"<?xml version=";}i:2;i:1;}i:3;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15;}i:4;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"1.0";}i:2;i:16;}i:5;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:19;}i:6;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:"?>
<robot name=";}i:2;i:20;}i:7;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:35;}i:8;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"my_robot";}i:2;i:36;}i:9;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44;}i:10;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:">";}i:2;i:45;}i:11;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:47;}i:12;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:75:"  <material name="grey">
      <color rgba="0.5 0.5 0.5 1" />
  </material>";}i:2;i:47;}i:13;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:140:"  <material name="blue">
      <color rgba="0 0 0.5 1" />
  </material>
  
  <!-- base_footprint (projection of base_link on the ground) -->";}i:2;i:130;}i:14;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:256:"  <link name="base_footprint"/>
  
  <joint name="base_joint" type="fixed">
      <parent link="base_footprint"/>
      <child link="base_link"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
  
  <!-- base_link and base_scan (required for Nav2) -->";}i:2;i:282;}i:15;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:937:"  <link name="base_link">
      <visual>
          <geometry>
              <box size="0.6 0.4 0.2" />
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <material name="blue" />
      </visual>
  </link>
  
  <joint name="base_scan_joint" type="fixed">
      <parent link="base_link"/>
      <child link="base_scan"/>
      <origin xyz="0 0 0.13" rpy="0 0 0"/>
  </joint>
  
  <link name="base_scan">
      <visual>
          <geometry>
              <cylinder radius="0.1" length="0.06"/>
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <material name="grey" />
      </visual>
  </link>
  
  <!-- Wheels (2 differential wheels + 1 caster wheel to keep the balance) -->
  
  <joint name="base_left_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="left_wheel"/>
      <origin xyz="-0.1 0.2 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
  </joint>";}i:2;i:558;}i:16;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:460:"  <link name="left_wheel">
      <visual>
          <geometry>
              <cylinder radius="0.1" length="0.05"/>
          </geometry>
          <origin xyz="0 0 0" rpy="1.57 0 0" />
          <material name="grey" />
      </visual>
  </link>
  
  <joint name="base_right_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="right_wheel"/>
      <origin xyz="-0.1 -0.2 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
  </joint>";}i:2;i:1565;}i:17;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:429:"  <link name="right_wheel">
      <visual>
          <geometry>
              <cylinder radius="0.1" length="0.05"/>
          </geometry>
          <origin xyz="0 0 0" rpy="1.57 0 0" />
          <material name="grey" />
      </visual>
  </link>
  
  <joint name="base_caster_wheel_joint" type="fixed">
      <parent link="base_link"/>
      <child link="caster_wheel"/>
      <origin xyz="0.2 0 -0.15" rpy="0 0 0"/>
  </joint>";}i:2;i:2059;}i:18;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:230:"  <link name="caster_wheel">
      <visual>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <material name="grey" />
      </visual>
  </link>";}i:2;i:2520;}i:19;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:2520;}i:20;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"</robot>";}i:2;i:2771;}i:21;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:2779;}i:22;a:3:{i:0;s:12:"document_end";i:1;a:0:{}i:2;i:2779;}}