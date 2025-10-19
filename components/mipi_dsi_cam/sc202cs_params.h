#pragma once

/**
 * ============================================
 * PARAMÈTRES CONFIGURABLES SC202CS
 * ============================================
 * 
 * Ce fichier centralise tous les paramètres de couleur,
 * exposition et éclairage pour le capteur SC202CS.
 * 
 * Modifiez ces valeurs selon votre environnement.
 */

namespace esphome {
namespace mipi_dsi_cam {
namespace sc202cs_params {

// ============================================
// EXPOSITION PAR DÉFAUT
// ============================================

// Valeur d'exposition initiale (0x4dc = 1244)
// - Plus bas = image plus sombre (ex: 0x300)
// - Plus haut = image plus claire (ex: 0x800)
constexpr uint16_t DEFAULT_EXPOSURE = 0x4dc;

// Limites d'exposition pour Auto Exposure
constexpr uint16_t MIN_EXPOSURE = 0x200;   // 512
constexpr uint16_t MAX_EXPOSURE = 0xF00;   // 3840

// ============================================
// GAIN PAR DÉFAUT
// ============================================

// Index de gain initial (0 = gain minimum 1.0x)
// - 0-31:   1.0x - 2.0x   (faible lumière)
// - 32-63:  2.0x - 4.0x   (moyen)
// - 64-127: 4.0x - 16.0x  (fort)
// - 128+:   16.0x - 64.0x (très fort, bruité)
constexpr uint8_t DEFAULT_GAIN_INDEX = 0;

// Limites de gain pour Auto Exposure
constexpr uint8_t MIN_GAIN_INDEX = 0;
constexpr uint8_t MAX_GAIN_INDEX = 120;  // Limité pour éviter trop de bruit

// ============================================
// BALANCE DES BLANCS (WHITE BALANCE)
// ============================================

// Gains de couleur (1.0 = neutre)
// 
// 🔥 ROUGE : Augmenter si image trop bleue/froide
constexpr float WB_RED_GAIN = 1.4f;

// 🔥 VERT : DIMINUER pour réduire la dominante verte (LED)
//           Valeur recommandée: 0.65 - 0.75 pour éclairage LED
constexpr float WB_GREEN_GAIN = 0.7f;

// 🔥 BLEU : Augmenter si image trop jaune/chaude
constexpr float WB_BLUE_GAIN = 1.2f;

// ============================================
// AUTO EXPOSURE (AE)
// ============================================

// Activer l'auto-exposition au démarrage
constexpr bool AUTO_EXPOSURE_ENABLED = true;

// Luminosité cible (0-255, 128 = moyen)
// - Plus bas = image plus sombre
// - Plus haut = image plus claire
constexpr uint8_t AE_TARGET_BRIGHTNESS = 128;

// Intervalle de mise à jour AE en millisecondes
// - Plus court = réaction plus rapide (mais plus de CPU)
// - Plus long = réaction plus lente (moins de CPU)
constexpr uint32_t AE_UPDATE_INTERVAL_MS = 200;

// Seuil avant ajustement (0-255)
// Ne corrige que si l'écart dépasse ce seuil
constexpr uint8_t AE_ADJUSTMENT_THRESHOLD = 15;

// Pas d'ajustement exposition
// Plus grand = changements plus rapides mais plus visibles
constexpr uint16_t AE_EXPOSURE_STEP = 0x80;  // 128

// Pas d'ajustement gain
constexpr uint8_t AE_GAIN_STEP = 4;

// ============================================
// PROFILES PRÉDÉFINIS
// ============================================

// Profile: Intérieur normal (bureaux, maison)
namespace profile_indoor_normal {
  constexpr uint16_t EXPOSURE = 0x4dc;  // 1244
  constexpr uint8_t GAIN = 20;          // ~2.0x
  constexpr float WB_RED = 1.4f;
  constexpr float WB_GREEN = 0.7f;
  constexpr float WB_BLUE = 1.2f;
}

// Profile: Intérieur sombre
namespace profile_indoor_dim {
  constexpr uint16_t EXPOSURE = 0x800;  // 2048
  constexpr uint8_t GAIN = 40;          // ~3.5x
  constexpr float WB_RED = 1.5f;
  constexpr float WB_GREEN = 0.75f;
  constexpr float WB_BLUE = 1.3f;
}

// Profile: Éclairage LED froid (bureaux modernes)
namespace profile_led_cool {
  constexpr uint16_t EXPOSURE = 0x500;  // 1280
  constexpr uint8_t GAIN = 24;          // ~2.2x
  constexpr float WB_RED = 1.45f;
  constexpr float WB_GREEN = 0.65f;     // Très réduit pour LED
  constexpr float WB_BLUE = 1.25f;
}

// Profile: Extérieur ensoleillé
namespace profile_outdoor_bright {
  constexpr uint16_t EXPOSURE = 0x300;  // 768
  constexpr uint8_t GAIN = 8;           // ~1.3x
  constexpr float WB_RED = 1.1f;
  constexpr float WB_GREEN = 0.85f;
  constexpr float WB_BLUE = 1.05f;
}

// Profile: Extérieur nuageux
namespace profile_outdoor_cloudy {
  constexpr uint16_t EXPOSURE = 0x600;  // 1536
  constexpr uint8_t GAIN = 16;          // ~1.7x
  constexpr float WB_RED = 1.2f;
  constexpr float WB_GREEN = 0.8f;
  constexpr float WB_BLUE = 1.15f;
}

}  // namespace sc202cs_params
}  // namespace mipi_dsi_cam
}  // namespace esphome
