#pragma once

/**
 * ============================================
 * PARAMÈTRES SC202CS - CORRECTIONS WB + FPS
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

constexpr uint8_t DEFAULT_GAIN_INDEX = 10;  // 🔥 Augmenté de 0 à 10 pour luminosité

constexpr uint8_t MIN_GAIN_INDEX = 0;
constexpr uint8_t MAX_GAIN_INDEX = 120;

// ============================================
// BALANCE DES BLANCS - 🔥 CORRIGÉ POUR LE VERT
// ============================================

// 🆕 Valeurs corrigées pour éliminer la dominante verte
// Si l'image reste verte, testez ces profils :

// PROFIL 1: Correction verte standard (essayez d'abord celui-ci)
constexpr float WB_RED_GAIN = 1.8f;    // ⬆️ Augmenté (avant: 1.4)
constexpr float WB_GREEN_GAIN = 0.5f;  // ⬇️ Réduit (avant: 0.7)
constexpr float WB_BLUE_GAIN = 1.6f;   // ⬆️ Augmenté (avant: 1.2)

// PROFIL 2: Si toujours vert, décommentez et testez :
// constexpr float WB_RED_GAIN = 2.0f;
// constexpr float WB_GREEN_GAIN = 0.4f;
// constexpr float WB_BLUE_GAIN = 1.8f;

// PROFIL 3: Correction extrême (si PROFIL 2 insuffisant) :
// constexpr float WB_RED_GAIN = 2.2f;
// constexpr float WB_GREEN_GAIN = 0.3f;
// constexpr float WB_BLUE_GAIN = 2.0f;

// ============================================
// AUTO EXPOSURE - 🚀 OPTIMISÉ POUR FLUIDITÉ
// ============================================

constexpr bool AUTO_EXPOSURE_ENABLED = true;

constexpr uint8_t AE_TARGET_BRIGHTNESS = 128;

// 🔥 Réduit l'overhead CPU pour améliorer les FPS
constexpr uint32_t AE_UPDATE_INTERVAL_MS = 1000;  // ⬆️ 1s au lieu de 500ms

constexpr uint8_t AE_ADJUSTMENT_THRESHOLD = 25;  // ⬆️ Augmenté

constexpr uint16_t AE_EXPOSURE_STEP = 0x100;  // ⬆️ Augmenté pour ajustements plus rapides
constexpr uint8_t AE_GAIN_STEP = 6;           // ⬆️ Augmenté

// ============================================
// 🆕 OPTIMISATIONS FPS
// ============================================

// Activer pour maximiser les FPS en désactivant l'AE en plein écran
constexpr bool DISABLE_AE_IN_FULLSCREEN = true;

// ============================================
// PROFILES PRÉDÉFINIS - AVEC CORRECTIONS WB
// ============================================

namespace profile_indoor_normal {
  constexpr uint16_t EXPOSURE = 0x4dc;
  constexpr uint8_t GAIN = 20;
  constexpr float WB_RED = 1.8f;
  constexpr float WB_GREEN = 0.5f;
  constexpr float WB_BLUE = 1.6f;
}

namespace profile_indoor_dim {
  constexpr uint16_t EXPOSURE = 0x800;
  constexpr uint8_t GAIN = 40;
  constexpr float WB_RED = 1.9f;
  constexpr float WB_GREEN = 0.5f;
  constexpr float WB_BLUE = 1.7f;
}

namespace profile_led_cool {
  constexpr uint16_t EXPOSURE = 0x500;
  constexpr uint8_t GAIN = 24;
  constexpr float WB_RED = 1.7f;
  constexpr float WB_GREEN = 0.45f;
  constexpr float WB_BLUE = 1.5f;
}

namespace profile_outdoor_bright {
  constexpr uint16_t EXPOSURE = 0x300;
  constexpr uint8_t GAIN = 8;
  constexpr float WB_RED = 1.4f;
  constexpr float WB_GREEN = 0.65f;
  constexpr float WB_BLUE = 1.3f;
}

namespace profile_outdoor_cloudy {
  constexpr uint16_t EXPOSURE = 0x600;
  constexpr uint8_t GAIN = 16;
  constexpr float WB_RED = 1.6f;
  constexpr float WB_GREEN = 0.55f;
  constexpr float WB_BLUE = 1.5f;
}

}  // namespace sc202cs_params
}  // namespace mipi_dsi_cam
}  // namespace esphome
