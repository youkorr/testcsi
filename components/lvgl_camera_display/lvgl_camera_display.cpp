#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "../mipi_dsi_cam/mipi_dsi_cam.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "🎥 LVGL Camera Display (Ultra Low Latency)");

  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "❌ Camera not configured");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "✅ Display initialized (zero-copy mode)");
}

void LVGLCameraDisplay::loop() {
  // Mode event-driven ultra-optimisé
  if (!this->camera_->is_streaming()) {
    return;
  }
  
  // Vérifier si nouvelle frame (atomique, non-bloquant)
  if (!this->camera_->has_new_frame()) {
    return;
  }
  
  // Capturer la frame (swap atomique)
  if (this->camera_->capture_frame()) {
    this->update_canvas_fast_();
    this->frame_count_++;

    // Logger FPS réel toutes les 100 frames
    if (this->frame_count_ % 100 == 0) {
      uint32_t now_time = millis();

      if (this->last_fps_time_ > 0) {
        float elapsed = (now_time - this->last_fps_time_) / 1000.0f;
        float fps = 100.0f / elapsed;
        ESP_LOGI(TAG, "🎞️ Display FPS: %.2f | %u frames total", fps, this->frame_count_);
      }
      this->last_fps_time_ = now_time;
    }
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Mode: Zero-copy + Direct invalidation");
  ESP_LOGCONFIG(TAG, "  Canvas: %s", this->canvas_obj_ ? "YES" : "NO");
}

// 🚀 VERSION ULTRA-OPTIMISÉE - Évite tout redraw inutile
void LVGLCameraDisplay::update_canvas_fast_() {
  if (this->camera_ == nullptr || this->canvas_obj_ == nullptr) {
    if (!this->canvas_warning_shown_) {
      ESP_LOGW(TAG, "❌ Canvas null");
      this->canvas_warning_shown_ = true;
    }
    return;
  }

  uint8_t* img_data = this->camera_->get_image_data();
  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();

  if (img_data == nullptr) {
    return;
  }

  if (this->first_update_) {
    ESP_LOGI(TAG, "🖼️  First canvas update:");
    ESP_LOGI(TAG, "   Dimensions: %ux%u", width, height);
    ESP_LOGI(TAG, "   Buffer: %p", img_data);
    this->first_update_ = false;
  }

  // 🔥 CRITIQUE: Ne JAMAIS appeler lv_canvas_set_buffer à chaque frame !
  // On le fait SEULEMENT si le pointeur change ou au premier update
  if (this->last_buffer_ptr_ != img_data) {
    lv_canvas_set_buffer(this->canvas_obj_, img_data, width, height, LV_IMG_CF_TRUE_COLOR);
    this->last_buffer_ptr_ = img_data;
    
    // Forcer un redraw complet seulement au changement de buffer
    lv_obj_invalidate(this->canvas_obj_);
  } else {
    // 🚀 OPTIMISATION CRITIQUE: Invalider SEULEMENT la zone image
    // Évite de redessiner les widgets autour
    lv_area_t area;
    area.x1 = lv_obj_get_x(this->canvas_obj_);
    area.y1 = lv_obj_get_y(this->canvas_obj_);
    area.x2 = area.x1 + width - 1;
    area.y2 = area.y1 + height - 1;
    
    // Invalider uniquement la zone du canvas
    lv_obj_invalidate_area(this->canvas_obj_, &area);
  }
}

void LVGLCameraDisplay::configure_canvas(lv_obj_t *canvas) { 
  this->canvas_obj_ = canvas;
  ESP_LOGI(TAG, "🎨 Canvas configured: %p", canvas);

  if (canvas != nullptr) {
    lv_coord_t w = lv_obj_get_width(canvas);
    lv_coord_t h = lv_obj_get_height(canvas);
    ESP_LOGI(TAG, "   Canvas size: %dx%d", w, h);
    
    // 🚀 Optimisations LVGL pour performance maximale
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    
    // 🔥 CRITIQUE: Désactiver le cache et le blending pour le canvas
    lv_obj_set_style_bg_opa(canvas, LV_OPA_TRANSP, 0);  // Pas de fond
    lv_obj_set_style_border_width(canvas, 0, 0);        // Pas de bordure (on la met ailleurs)
    lv_obj_set_style_pad_all(canvas, 0, 0);             // Pas de padding
    
    // Désactiver les animations
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  }
}

}  // namespace lvgl_camera_display
}  // namespace esphome
