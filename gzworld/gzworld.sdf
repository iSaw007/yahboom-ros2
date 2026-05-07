<sdf version='1.9'>
  <world name='default'>
    <plugin name='ignition::gazebo::systems::Physics' filename='libignition-gazebo-physics-system.so'/>
    <plugin name='ignition::gazebo::systems::UserCommands' filename='libignition-gazebo-user-commands-system.so'/>
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='libignition-gazebo-scene-broadcaster-system.so'/>
    <plugin name='ignition::gazebo::systems::Imu' filename='ignition-gazebo-imu-system'/>
    <plugin name='ignition::gazebo::systems::Sensors' filename='ignition-gazebo-sensors-system'>
      <render_engine>ogre</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.01 0.01 0.01 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <include>
      <uri>file://<urdf-string></uri>
      <name>yahboomcar</name>
      <pose>4.97558 2.58 0.045249 -4e-06 -1.1e-05 -1.1089</pose>
    </include>
    <model name='box'>
      <pose>1.35832 0.027556 0.5 0 0 -0</pose>
      <link name='box_link'>
        <inertial>
          <inertia>
            <ixx>0.16666</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.16666</iyy>
            <iyz>0</iyz>
            <izz>0.16666</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='box_collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='box_visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='cylinder'>
      <pose>-0.115543 -1.91985 0.499999 0 0 0</pose>
      <link name='cylinder_link'>
        <inertial>
          <inertia>
            <ixx>0.14580000000000001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.14580000000000001</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='cylinder_collision'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='cylinder_visual'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='capsule'>
      <pose>2.76257 -0.19079 0.5 0 -0 0</pose>
      <link name='capsule_link'>
        <inertial>
          <inertia>
            <ixx>0.074153999999999998</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.074153999999999998</iyy>
            <iyz>0</iyz>
            <izz>0.018769000000000001</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='capsule_collision'>
          <geometry>
            <capsule>
              <radius>0.20000000000000001</radius>
              <length>0.59999999999999998</length>
            </capsule>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='capsule_visual'>
          <geometry>
            <capsule>
              <radius>0.20000000000000001</radius>
              <length>0.59999999999999998</length>
            </capsule>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='capsule_0'>
      <pose>0.279321 2.1794 0.5 0 -0 0</pose>
      <link name='capsule_link'>
        <inertial>
          <inertia>
            <ixx>0.074153999999999998</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.074153999999999998</iyy>
            <iyz>0</iyz>
            <izz>0.018769000000000001</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='capsule_collision'>
          <geometry>
            <capsule>
              <radius>0.20000000000000001</radius>
              <length>0.59999999999999998</length>
            </capsule>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='capsule_visual'>
          <geometry>
            <capsule>
              <radius>0.20000000000000001</radius>
              <length>0.59999999999999998</length>
            </capsule>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='cylinder_0'>
      <pose>-3.01817 1.49164 0.499999 -0 -0 -0</pose>
      <link name='cylinder_link'>
        <inertial>
          <inertia>
            <ixx>0.14580000000000001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.14580000000000001</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='cylinder_collision'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='cylinder_visual'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='box_0'>
      <pose>-2.80865 -1.85104 0.499999 0 -1e-06 0.027943</pose>
      <link name='box_link'>
        <inertial>
          <inertia>
            <ixx>0.16666</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.16666</iyy>
            <iyz>0</iyz>
            <izz>0.16666</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='box_collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='box_visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='cylinder_1'>
      <pose>-1.77313 3.10015 0.499999 0 0 0</pose>
      <link name='cylinder_link'>
        <inertial>
          <inertia>
            <ixx>0.14580000000000001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.14580000000000001</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='cylinder_collision'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='cylinder_visual'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <model name='box_1'>
      <pose>2.58902 2.82114 0.5 0 0 -0</pose>
      <link name='box_link'>
        <inertial>
          <inertia>
            <ixx>0.16666</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.16666</iyy>
            <iyz>0</iyz>
            <izz>0.16666</izz>
          </inertia>
          <mass>1</mass>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='box_collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='box_visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
    <light name='sun' type='directional'>
      <pose>0 0 10 0 -0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <linear>1</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
  </world>
</sdf>
