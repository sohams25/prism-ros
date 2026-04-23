# Contributing to Prism

Thanks for your interest in contributing.

## Commit conventions

Prism follows [Conventional Commits](https://www.conventionalcommits.org/)
as of v0.1.0. Commit subjects look like:

    <type>[optional scope]: <description>

Where `<type>` is one of:

- `feat` — a new user-facing feature
- `fix` — a bug fix
- `refactor` — code change that is neither a feature nor a fix
- `docs` — documentation only
- `test` — adding or fixing tests
- `ci` — continuous-integration configuration
- `chore` — tooling, build, or housekeeping

Breaking changes append a `BREAKING CHANGE:` footer explaining
the break. This drives the SemVer bump.

## Workflow

1. Fork and branch from `main`.
2. Make your change with tests.
3. Run `colcon build --packages-select prism_image_proc` and
   `colcon test --packages-select prism_image_proc` locally.
   Both must be green.
4. Open a pull request describing the change in plain English.
   Reference any issue it closes.
5. CI must pass before merge.

## Running the test suite

    cd ~/ros2_ws
    colcon build --packages-select prism_image_proc
    source install/setup.bash
    colcon test --packages-select prism_image_proc \
      --ctest-args -R test_direct_resize
    colcon test-result --verbose

## Scope of contributions

Prism targets the segment of the ROS 2 fleet Isaac ROS does not
cover: Intel iGPU, AMD, older Jetson, Jetson Orin on Humble, and
non-NVIDIA embedded hosts. Contributions that expand backend
coverage within that scope, or that broaden the action registry
(new chainable image ops), are especially welcome.

## Reporting bugs

Open an issue with: ROS 2 distribution and OS, GStreamer version
(`gst-inspect-1.0 --version`), hardware, the launch command you
ran, and the behavior you observed. Include logs — stderr from
the component container is the most useful.

## License

By contributing you agree your contributions are licensed under
Apache-2.0, matching the project.
