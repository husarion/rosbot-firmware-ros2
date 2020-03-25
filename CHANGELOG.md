# ROSbot ros2 firmware CHANGELOG

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) and this project adheres to [Semantic Versioning](http://semver.org/).

## [0.1.0] - 2020-03-13

First fully working release version. Available topics:
* `/odom`
* `/battery`
* `/cmd_vel`

### Added
  - `now()` timestamp (without time synchronization)

### Fixed 
  - Fixed bug in Micro-XRCE-DDS serialization function.   

## [0.2.0] - 2019-03-23

### Added
  - new `tf2_msgs::TFMessage` message type
  - `/tf` broadcaster with `tf2_msgs::TFMessage` message type
  - time synchronization using `/rosbot_time` subscriber