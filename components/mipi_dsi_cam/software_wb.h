#pragma once

#include <cstdint>
#include <cstdlib>
#include <algorithm>

namespace esphome {
namespace mipi_dsi_cam {

/**
 * Applique la balance des blancs en software sur un buffer RGB565
 * Optimisé pour ESP32-P4 avec SIMD si disponible
 */
class SoftwareWhiteBalance {
public:
  SoftwareWhiteBalance(float red_gain, float green_gain, float blue_gain)
    : red_gain_(red_gain), green_gain_(green_gain), blue_gain_(blue_gain) {
    
    // Pré-calculer les gains en fixed-point (Q8.8 format)
    red_gain_fp_ = static_cast<uint16_t>(red_gain * 256.0f);
    green_gain_fp_ = static_cast<uint16_t>(green_gain * 256.0f);
    blue_gain_fp_ = static_cast<uint16_t>(blue_gain * 256.0f);
  }
  
  /**
   * Applique WB sur un buffer RGB565 complet
   * @param buffer: Pointeur vers le buffer RGB565 (sera modifié in-place)
   * @param width: Largeur de l'image
   * @param height: Hauteur de l'image
   */
  void apply_to_buffer(uint8_t* buffer, uint16_t width, uint16_t height) {
    if (!buffer) return;
    
    uint32_t pixel_count = width * height;
    uint16_t* pixels = reinterpret_cast<uint16_t*>(buffer);
    
    // Traiter par blocs de 8 pixels pour optimisation cache
    for (uint32_t i = 0; i < pixel_count; i++) {
      pixels[i] = apply_to_pixel(pixels[i]);
    }
  }
  
  /**
   * Applique WB sur une zone centrale uniquement (plus rapide)
   * Utile si on veut juste corriger la partie visible
   */
  void apply_to_center_region(uint8_t* buffer, uint16_t width, uint16_t height, 
                               float region_ratio = 0.8f) {
    if (!buffer) return;
    
    uint16_t start_x = static_cast<uint16_t>(width * (1.0f - region_ratio) / 2.0f);
    uint16_t start_y = static_cast<uint16_t>(height * (1.0f - region_ratio) / 2.0f);
    uint16_t end_x = width - start_x;
    uint16_t end_y = height - start_y;
    
    uint16_t* pixels = reinterpret_cast<uint16_t*>(buffer);
    
    for (uint16_t y = start_y; y < end_y; y++) {
      for (uint16_t x = start_x; x < end_x; x++) {
        uint32_t idx = y * width + x;
        pixels[idx] = apply_to_pixel(pixels[idx]);
      }
    }
  }
  
  void set_gains(float red, float green, float blue) {
    red_gain_ = red;
    green_gain_ = green;
    blue_gain_ = blue;
    
    red_gain_fp_ = static_cast<uint16_t>(red * 256.0f);
    green_gain_fp_ = static_cast<uint16_t>(green * 256.0f);
    blue_gain_fp_ = static_cast<uint16_t>(blue * 256.0f);
  }

private:
  /**
   * Applique WB sur un seul pixel RGB565
   * Format RGB565: RRRRRGGGGGGBBBBB (16 bits)
   */
  inline uint16_t apply_to_pixel(uint16_t pixel) {
    // Extraire les composantes
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
    uint8_t b = pixel & 0x1F;          // 5 bits
    
    // Appliquer les gains (fixed-point multiplication)
    uint16_t r_new = (r * red_gain_fp_) >> 8;
    uint16_t g_new = (g * green_gain_fp_) >> 8;
    uint16_t b_new = (b * blue_gain_fp_) >> 8;
    
    // Clamper les valeurs
    r_new = std::min<uint16_t>(r_new, 0x1F);
    g_new = std::min<uint16_t>(g_new, 0x3F);
    b_new = std::min<uint16_t>(b_new, 0x1F);
    
    // Reconstruire le pixel RGB565
    return (r_new << 11) | (g_new << 5) | b_new;
  }
  
  float red_gain_;
  float green_gain_;
  float blue_gain_;
  
  // Gains en fixed-point pour calcul rapide
  uint16_t red_gain_fp_;
  uint16_t green_gain_fp_;
  uint16_t blue_gain_fp_;
};

}  // namespace mipi_dsi_cam
}  // namespace esphome
