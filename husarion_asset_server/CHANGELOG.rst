^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husarion_asset_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.3 (2026-08-26)
------------------

1.2.2 (2026-08-25)
------------------

1.2.1 (2026-08-14)
------------------
* Bump asset_server + retry download (`#191 <https://github.com/husarion/rosbot_ros/issues/191>`_)
* Contributors: Rafal Gorecki

1.2.0 (2026-08-12)
------------------
* Merge pull request `#187 <https://github.com/husarion/rosbot_ros/issues/187>`_ from husarion/feature/asset-server-bringup
  Run husarion_asset_server in the driver launch, not a separate daemon
* Run husarion_asset_server in the driver launch, not a separate daemon
  Vendors it as a new husarion_asset_server package that fetches +
  sha256-verifies the prebuilt release binary instead of building via
  colcon-ros-cargo (avoids a Rust/clang toolchain in dev/CI/rosbot-snap).
  Runs as a node in rosbot_bringup's own launch under push_ros_namespace,
  gated by a new asset_server arg (default True) - gets the correct
  namespace at startup instead of needing a restart-to-re-announce hack.
* Contributors: Rafal Gorecki, rafal-gorecki
