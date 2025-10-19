#pragma once

namespace esphome {
namespace mipi_dsi_cam {
namespace sc202cs_params {

// Valeurs raisonnables avec esp_video qui gère le pipeline
constexpr uint16_t DEFAULT_EXPOSURE = 0x0800;  // Plus élevé
constexpr uint8_t DEFAULT_GAIN_INDEX = 20;     // Modéré

constexpr uint16_t MIN_EXPOSURE = 0x200;
constexpr uint16_t MAX_EXPOSURE = 0x0E00;

constexpr uint8_t MIN_GAIN_INDEX = 0;
constexpr uint8_t MAX_GAIN_INDEX = 100;

constexpr bool AUTO_EXPOSURE_ENABLED = true;
constexpr uint8_t AE_TARGET_BRIGHTNESS = 128;
constexpr uint32_t AE_UPDATE_INTERVAL_MS = 1000;
constexpr uint8_t AE_ADJUSTMENT_THRESHOLD = 20;
constexpr uint16_t AE_EXPOSURE_STEP = 0x100;
constexpr uint8_t AE_GAIN_STEP = 5;

}  // namespace sc202cs_params
}  // namespace mipi_dsi_cam
}  // namespace esphome
