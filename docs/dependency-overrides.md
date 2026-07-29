# Dependency override register

Every override is treated as part of the release-qualified environment and
must be re-evaluated when Isaac Sim or Isaac Lab changes.

## `coverage==7.4.4`

Isaac Sim 6.0.1.0 declares `coverage==7.4.4` through
`isaacsim-kernel==6.0.1.0`. The pinned Isaac Lab commit declares
`coverage==7.6.1` as a required dependency even though it is used only by its
test suite. Those exact requirements cannot be solved together.

The environment follows the runtime product's exact pin and uses uv's
`override-dependencies` facility. No source file in the pinned Isaac Lab
checkout is patched, so its Git commit and tree remain independently
verifiable.
