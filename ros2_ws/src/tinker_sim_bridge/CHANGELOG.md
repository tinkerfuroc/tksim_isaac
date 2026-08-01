# Changelog

All notable changes to this package are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Canonical manipulation model-bundle producer (`model_bundle`) and bounded
  preflight validator (`model_preflight`), with pure ROS-free
  `model_contract` semantics matching the production `xarm_moveit_config`
  consumer.  Registers the `model_bundle` and `model_preflight` console
  scripts, declares the direct interface dependencies used by the model
  overlay, and documents the schema/producer/preflight contract in the README.
