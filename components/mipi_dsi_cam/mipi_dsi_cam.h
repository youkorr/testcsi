#pragma once

/**
 * ============================================
 * PARAMÈTRES SC202CS - VERSION OPTIMISÉE FPS
 * ============================================
 */

namespace esphome {
namespace mipi_dsi_cam {
namespace sc202cs_params {

// ============================================
// EXPOSITION PAR DÉFAUT
// ============================================

constexpr uint16_t DEFAULT_EXPOSURE = 0x4dc;

constexpr uint16_t MIN_EXPOSURE = 0x200;
constexpr uint16_t MAX_EXPOSURE = 0xF00;

// ============================================
// GAIN PAR DÉFAUT
// ============================================

constexpr uint8_t DEFAULT_GAIN_INDEX = 0;

constexpr uint8_t MIN_GAIN_INDEX = 0;
constexpr uint8_t MAX_GAIN_INDEX = 120;

// ============================================
// BALANCE DES BLANCS
// ============================================

constexpr float WB_RED_GAIN = 1.4f;
constexpr float WB_GREEN_GAIN = 0.7f;
constexpr float WB_BLUE_GAIN = 1.2f;

// ============================================
// AUTO EXPOSURE - 🚀 OPTIMISÉ POUR FPS
// ============================================

constexpr bool AUTO_EXPOSURE_ENABLED = true;

constexpr uint8_t AE_TARGET_BRIGHTNESS = 128;

// 🔥 CRITIQUE: Augmenté de 200ms à 500ms pour réduire la charge CPU
// Mise à jour AE 2x par seconde au lieu de 5x
constexpr uint32_t AE_UPDATE_INTERVAL_MS = 500;

// 🔥 Seuil augmenté pour éviter les ajustements trop fréquents
// Ne corrige que si l'écart dépasse 20 au lieu de 15
constexpr uint8_t AE_ADJUSTMENT_THRESHOLD = 20;

// Pas d'ajustement - plus grands pour des changements moins fréquents
constexpr uint16_t AE_EXPOSURE_STEP = 0xA0;  // 160 (au lieu de 128)
constexpr uint8_t AE_GAIN_STEP = 5;          // 5 (au lieu de 4)

// ============================================
// 🆕 OPTION: DÉSACTIVER AE EN PLEIN ÉCRAN
// ============================================

// Si activé, l'AE se désactive automatiquement en mode plein écran
// pour maximiser les FPS (à activer dans le code si besoin)
constexpr bool DISABLE_AE_IN_FULLSCREEN = false;

// ============================================
// PROFILES PRÉDÉFINIS (inchangés)
// ============================================

namespace profile_indoor_normal {
  constexpr uint16_t EXPOSURE = 0x4dc;
  constexpr uint8_t GAIN = 20;
  constexpr float WB_RED = 1.4f;
  constexpr float WB_GREEN = 0.7f;
  constexpr float WB_BLUE = 1.2f;
}

namespace profile_indoor_dim {
  constexpr uint16_t EXPOSURE = 0x800;
  constexpr uint8_t GAIN = 40;
  constexpr float WB_RED = 1.5f;
  constexpr float WB_GREEN = 0.75f;
  constexpr float WB_BLUE = 1.3f;
}

namespace profile_led_cool {
  constexpr uint16_t EXPOSURE = 0x500;
  constexpr uint8_t GAIN = 24;
  constexpr float WB_RED = 1.45f;
  constexpr float WB_GREEN = 0.65f;
  constexpr float WB_BLUE = 1.25f;
}

namespace profile_outdoor_bright {
  constexpr uint16_t EXPOSURE = 0x300;
  constexpr uint8_t GAIN = 8;
  constexpr float WB_RED = 1.1f;
  constexpr float WB_GREEN = 0.85f;
  constexpr float WB_BLUE = 1.05f;
}

namespace profile_outdoor_cloudy {
  constexpr uint16_t EXPOSURE = 0x600;
  constexpr uint8_t GAIN = 16;
  constexpr float WB_RED = 1.2f;
  constexpr float WB_GREEN = 0.8f;
  constexpr float WB_BLUE = 1.15f;
}

}  // namespace sc202cs_params
}  // namespace mipi_dsi_cam
}  // namespace esphome
