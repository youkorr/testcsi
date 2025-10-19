#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_esphome.h"
#include "../mipi_dsi_cam/mipi_dsi_cam.h"

namespace esphome {
namespace lvgl_camera_display {

class LVGLCameraDisplay : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_camera(mipi_dsi_cam::MipiDsiCam *camera) { this->camera_ = camera; }
  void set_canvas_id(const std::string &canvas_id) { this->canvas_id_ = canvas_id; }
  void set_update_interval(uint32_t interval_ms) { 
    // Ignoré en mode event-driven
    this->update_interval_ = interval_ms; 
  }

  void configure_canvas(lv_obj_t *canvas);

  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  mipi_dsi_cam::MipiDsiCam *camera_{nullptr};
  lv_obj_t *canvas_obj_{nullptr};
  std::string canvas_id_{};

  uint32_t update_interval_{33};  // Non utilisé en mode event-driven
  uint32_t last_update_{0};       // Non utilisé en mode event-driven

  uint32_t frame_count_{0};
  bool first_update_{true};
  bool canvas_warning_shown_{false};

  uint32_t last_fps_time_{0};
  
  // Suivi du pointeur de buffer pour éviter les appels inutiles
  uint8_t* last_buffer_ptr_{nullptr};

  // 🚀 Version optimisée du update canvas
  void update_canvas_fast_();
  
  // Ancienne version (gardée pour compatibilité si besoin)
  void update_canvas_() { update_canvas_fast_(); }
};

}  // namespace lvgl_camera_display
}  // namespace esphome
