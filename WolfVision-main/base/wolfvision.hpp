#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/core.hpp>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/angle_solve/basic_pnp.hpp"
#include "utils/fps.hpp"
#include "utils/reset_mv_camera.hpp"

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "wolfvision");
