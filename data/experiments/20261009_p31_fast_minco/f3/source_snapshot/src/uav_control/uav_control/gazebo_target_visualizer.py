"""Gazebo visualization helper for the simulated moving target."""

import math
import re
import time


_VALID_ENTITY_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_-]*$')


def ned_to_gazebo_enu(x, y, z):
    """Convert PX4 local NED coordinates to Gazebo world ENU."""
    return float(y), float(x), -float(z)


def red_sphere_sdf(entity_name, diameter):
    """Return a static, visual-only red sphere model."""
    if not _VALID_ENTITY_NAME.fullmatch(entity_name):
        raise ValueError(
            'Gazebo entity name must start with a letter and contain only '
            'letters, numbers, underscores, or hyphens.'
        )
    diameter = float(diameter)
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ValueError('Gazebo target sphere diameter must be positive.')

    radius = 0.5 * diameter
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{entity_name}">
    <static>true</static>
    <link name="target_link">
      <visual name="target_visual">
        <cast_shadows>true</cast_shadows>
        <geometry>
          <sphere>
            <radius>{radius:.9g}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''


class GazeboTargetVisualizer:
    """Create and move a visual-only sphere through Gazebo Transport."""

    def __init__(
        self,
        world_name='default',
        entity_name='usv_target',
        diameter=0.5,
        request_timeout_ms=100,
        create_retry_period=1.0,
    ):
        # Keep Gazebo imports optional so the target generator can still run
        # in ROS-only and unit-test environments.
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node as GazeboTransportNode

        self._boolean_type = Boolean
        self._entity_factory_type = EntityFactory
        self._pose_type = Pose
        self._world_control_type = WorldControl
        self._node = GazeboTransportNode()

        self._world_name = str(world_name).strip().strip('/')
        if not self._world_name:
            raise ValueError('Gazebo world name must not be empty.')
        self._entity_name = str(entity_name).strip()
        self._model_sdf = red_sphere_sdf(
            self._entity_name,
            diameter,
        )
        self._request_timeout_ms = max(int(request_timeout_ms), 1)
        self._create_retry_period = max(float(create_retry_period), 0.1)
        self._create_service = (
            f'/world/{self._world_name}/create'
        )
        self._set_pose_service = (
            f'/world/{self._world_name}/set_pose'
        )
        self._control_service = (
            f'/world/{self._world_name}/control'
        )
        self._created = False
        self._last_create_attempt = -math.inf
        self.last_error = ''
        self._world_paused = None
        if not self._node.subscribe(
            WorldStatistics, f'/world/{self._world_name}/stats', self._world_stats_callback,
        ):
            raise RuntimeError('Could not subscribe to native Gazebo pause state.')

    @property
    def world_paused(self):
        """Native world pause state, unknown until the first statistics packet."""
        return self._world_paused

    def _world_stats_callback(self, message):
        self._world_paused = bool(message.paused)

    @property
    def created(self):
        return self._created

    def _request(self, service, request, request_type):
        executed, response = self._node.request(
            service,
            request,
            request_type,
            self._boolean_type,
            self._request_timeout_ms,
        )
        return bool(executed and response.data)

    def _set_pose(self, ned_x, ned_y, ned_z):
        gazebo_x, gazebo_y, gazebo_z = ned_to_gazebo_enu(
            ned_x,
            ned_y,
            ned_z,
        )
        pose = self._pose_type()
        pose.name = self._entity_name
        pose.position.x = gazebo_x
        pose.position.y = gazebo_y
        pose.position.z = gazebo_z
        pose.orientation.w = 1.0
        return self._request(
            self._set_pose_service,
            pose,
            self._pose_type,
        )

    def update(self, ned_x, ned_y, ned_z):
        """Ensure the sphere exists, then synchronize its NED position."""
        now = time.monotonic()
        if not self._created:
            if now - self._last_create_attempt < self._create_retry_period:
                return False
            self._last_create_attempt = now

            factory = self._entity_factory_type()
            factory.name = self._entity_name
            factory.sdf = self._model_sdf
            factory.allow_renaming = False

            if self._request(
                self._create_service,
                factory,
                self._entity_factory_type,
            ):
                self._created = True
            elif self._set_pose(ned_x, ned_y, ned_z):
                # Reuse a sphere left in the world by a previous node run.
                self._created = True
            else:
                self.last_error = (
                    f'Gazebo services unavailable: {self._create_service}'
                )
                return False

        if not self._set_pose(ned_x, ned_y, ned_z):
            self.last_error = (
                f'Gazebo pose update failed: {self._set_pose_service}'
            )
            self._created = False
            return False

        self.last_error = ''
        return True

    def pause_world(self):
        """Pause Gazebo so the successful interception stays frozen."""
        control = self._world_control_type()
        control.pause = True
        if not self._request(
            self._control_service,
            control,
            self._world_control_type,
        ):
            self.last_error = (
                f'Gazebo world pause failed: {self._control_service}'
            )
            return False

        self.last_error = ''
        return True
