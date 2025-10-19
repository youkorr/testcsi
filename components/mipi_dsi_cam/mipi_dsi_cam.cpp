#include "mipi_dsi_cam.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include "mipi_dsi_cam_drivers_generated.h"
#include "sc202cs_params.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// 🆕 esp_video includes
#include "esp_video_init.h"
#include "esp_video_venc.h"

namespace esphome {
namespace mipi_dsi_cam {

static const char *const TAG = "mipi_dsi_cam";

void MipiDsiCam::setup() {
  ESP_LOGI(TAG, "Init MIPI Camera with esp_video pipeline");
  ESP_LOGI(TAG, "  Sensor type: %s", this->sensor_type_.c_str());
  
  // Charger paramètres
  this->current_exposure_ = sc202cs_params::DEFAULT_EXPOSURE;
  this->current_gain_index_ = sc202cs_params::DEFAULT_GAIN_INDEX;
  this->ae_target_brightness_ = sc202cs_params::AE_TARGET_BRIGHTNESS;
  this->auto_exposure_enabled_ = sc202cs_params::AUTO_EXPOSURE_ENABLED;
  
  ESP_LOGI(TAG, "🔥 SC202CS Config:");
  ESP_LOGI(TAG, "   Exposure: 0x%04X", this->current_exposure_);
  ESP_LOGI(TAG, "   Gain: %d", this->current_gain_index_);
  ESP_LOGI(TAG, "   AE: %s", this->auto_exposure_enabled_ ? "ON" : "OFF");
  
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
  
  if (!this->create_sensor_driver_()) {
    ESP_LOGE(TAG, "Driver creation failed");
    this->mark_failed();
    return;
  }
  
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Sensor init failed");
    this->mark_failed();
    return;
  }
  
  if (this->has_external_clock()) {
    if (!this->init_external_clock_()) {
      ESP_LOGE(TAG, "External clock init failed");
      this->mark_failed();
      return;
    }
  }
  
  // 🆕 Initialiser le pipeline esp_video au lieu du LDO/CSI/ISP manuels
  if (!this->init_esp_video_pipeline_()) {
    ESP_LOGE(TAG, "esp_video pipeline init failed");
    this->mark_failed();
    return;
  }
  
  // Créer la file de commandes AE
  this->ae_command_queue_ = xQueueCreate(4, sizeof(AECommand));
  if (!this->ae_command_queue_) {
    ESP_LOGE(TAG, "AE queue creation failed");
    this->mark_failed();
    return;
  }
  
  xTaskCreatePinnedToCore(
    ae_task_,
    "ae_task",
    3072,
    this,
    1,
    &this->ae_task_handle_,
    0
  );
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Camera ready - esp_video pipeline active");
}

bool MipiDsiCam::create_sensor_driver_() {
  ESP_LOGI(TAG, "Creating driver for: %s", this->sensor_type_.c_str());
  
  this->sensor_driver_ = create_sensor_driver(this->sensor_type_, this);
  
  if (this->sensor_driver_ == nullptr) {
    ESP_LOGE(TAG, "Unknown sensor: %s", this->sensor_type_.c_str());
    return false;
  }
  
  ESP_LOGI(TAG, "Driver created for: %s", this->sensor_driver_->get_name());
  return true;
}

bool MipiDsiCam::init_sensor_() {
  if (!this->sensor_driver_) {
    ESP_LOGE(TAG, "No sensor driver");
    return false;
  }
  
  ESP_LOGI(TAG, "Init sensor: %s", this->sensor_driver_->get_name());
  
  this->width_ = this->sensor_driver_->get_width();
  this->height_ = this->sensor_driver_->get_height();
  this->lane_count_ = this->sensor_driver_->get_lane_count();
  this->bayer_pattern_ = this->sensor_driver_->get_bayer_pattern();
  this->lane_bitrate_mbps_ = this->sensor_driver_->get_lane_bitrate_mbps();
  
  ESP_LOGI(TAG, "  Resolution: %ux%u", this->width_, this->height_);
  ESP_LOGI(TAG, "  Lanes: %u", this->lane_count_);
  ESP_LOGI(TAG, "  Bayer: %u", this->bayer_pattern_);
  ESP_LOGI(TAG, "  Bitrate: %u Mbps", this->lane_bitrate_mbps_);
  
  uint16_t pid = 0;
  esp_err_t ret = this->sensor_driver_->read_id(&pid);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read sensor ID");
    return false;
  }
  
  if (pid != this->sensor_driver_->get_pid()) {
    ESP_LOGE(TAG, "Wrong PID: 0x%04X (expected 0x%04X)", 
             pid, this->sensor_driver_->get_pid());
    return false;
  }
  
  ESP_LOGI(TAG, "Sensor ID: 0x%04X", pid);
  
  ret = this->sensor_driver_->init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Sensor init failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "Sensor initialized");
  delay(200);
  
  return true;
}

bool MipiDsiCam::init_external_clock_() {
  ESP_LOGI(TAG, "Init external clock on GPIO%d @ %u Hz", 
           this->external_clock_pin_, this->external_clock_frequency_);
  
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->external_clock_frequency_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  
  esp_err_t ret = ledc_timer_config(&ledc_timer);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer config failed: %d", ret);
    return false;
  }
  
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = this->external_clock_pin_;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = 1;
  ledc_channel.hpoint = 0;
  
  ret = ledc_channel_config(&ledc_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LEDC channel config failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "External clock initialized");
  return true;
}

// 🆕 Initialiser le pipeline esp_video
bool MipiDsiCam::init_esp_video_pipeline_() {
  ESP_LOGI(TAG, "Init esp_video pipeline");
  
  // Configuration du pipeline vidéo
  esp_video_init_config_t video_config = {
    .csi = {
      .ctlr_id = 0,
      .lane_bit_rate_mbps = this->lane_bitrate_mbps_,
      .data_lane_num = this->lane_count_,
      .h_res = this->width_,
      .v_res = this->height_,
      .byte_swap_en = false,
      .queue_items = 3,
    },
    .isp = {
      .clk_hz = 120000000,
      .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
      .input_data_color_type = ISP_COLOR_RAW8,
      .output_data_color_type = ISP_COLOR_RGB565,
      .h_res = this->width_,
      .v_res = this->height_,
      .bayer_order = (color_raw_element_order_t)this->bayer_pattern_,
      .has_line_start_packet = false,
      .has_line_end_packet = false,
    },
    .buffer_num = 3,  // Triple buffering
  };
  
  // Initialiser le pipeline
  esp_err_t ret = esp_video_init(&video_config, &this->video_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: 0x%x", ret);
    return false;
  }
  
  // Récupérer le handle ISP pour AWB/AE
  this->isp_handle_ = esp_video_get_isp_handle(this->video_handle_);
  if (!this->isp_handle_) {
    ESP_LOGW(TAG, "Could not get ISP handle");
  } else {
    this->configure_isp_features_();
  }
  
  ESP_LOGI(TAG, "✅ esp_video pipeline initialized");
  return true;
}

void MipiDsiCam::configure_isp_features_() {
  if (!this->isp_handle_) return;
  
  // Configurer AWB si disponible
  esp_isp_awb_config_t awb_config = {};
  awb_config.sample_point = ISP_AWB_SAMPLE_POINT_AFTER_CCM;
  awb_config.window.top_left.x = this->width_ / 4;
  awb_config.window.top_left.y = this->height_ / 4;
  awb_config.window.btm_right.x = (this->width_ * 3) / 4;
  awb_config.window.btm_right.y = (this->height_ * 3) / 4;
  
  esp_err_t ret = esp_isp_new_awb_controller(this->isp_handle_, &awb_config, &this->awb_ctlr_);
  
  if (ret == ESP_OK && this->awb_ctlr_ != nullptr) {
    esp_isp_awb_controller_enable(this->awb_ctlr_);
    ESP_LOGI(TAG, "✅ AWB matériel activé via esp_video");
  } else {
    ESP_LOGW(TAG, "AWB not available (0x%x)", ret);
  }
}

bool MipiDsiCam::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Start streaming");
  
  this->total_frames_received_ = 0;
  this->last_frame_log_time_ = millis();
  
  // Démarrer le sensor
  if (this->sensor_driver_) {
    esp_err_t ret = this->sensor_driver_->start_stream();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Sensor start failed: %d", ret);
      return false;
    }
    delay(100);
  }
  
  // Démarrer le pipeline esp_video
  esp_err_t ret = esp_video_start(this->video_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_start failed: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming active (esp_video)");
  return true;
}

bool MipiDsiCam::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_video_stop(this->video_handle_);
  
  if (this->sensor_driver_) {
    this->sensor_driver_->stop_stream();
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "Streaming stopped");
  return true;
}

bool MipiDsiCam::capture_frame() {
  if (!this->streaming_) {
    return false;
  }
  
  // Obtenir une frame depuis esp_video
  esp_video_frame_t *frame = nullptr;
  esp_err_t ret = esp_video_get_frame(this->video_handle_, &frame, 0);
  
  if (ret != ESP_OK || !frame) {
    return false;
  }
  
  // Mettre à jour le pointeur de buffer
  this->display_buffer_ = (uint8_t*)frame->data;
  this->frame_buffer_size_ = frame->size;
  
  // Marquer comme prêt
  this->frame_number_++;
  this->total_frames_received_++;
  
  // Libérer la frame (esp_video gère le buffer)
  esp_video_release_frame(this->video_handle_, frame);
  
  return true;
}

// Tâche AE asynchrone
void MipiDsiCam::ae_task_(void* param) {
  MipiDsiCam* cam = (MipiDsiCam*)param;
  AECommand cmd;
  
  ESP_LOGI(TAG, "AE task started");
  
  while (true) {
    if (xQueueReceive(cam->ae_command_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
      if (cam->sensor_driver_) {
        cam->sensor_driver_->set_exposure(cmd.exposure);
        vTaskDelay(pdMS_TO_TICKS(5));
        cam->sensor_driver_->set_gain(cmd.gain);
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void MipiDsiCam::update_auto_exposure_() {
  if (!this->auto_exposure_enabled_ || !this->sensor_driver_) {
    return;
  }
  
  uint32_t now = millis();
  if (now - this->last_ae_update_ < sc202cs_params::AE_UPDATE_INTERVAL_MS) {
    return;
  }
  this->last_ae_update_ = now;
  
  uint32_t avg_brightness = this->calculate_brightness_();
  int32_t error = (int32_t)this->ae_target_brightness_ - (int32_t)avg_brightness;
  
  if (abs(error) > sc202cs_params::AE_ADJUSTMENT_THRESHOLD) {
    bool changed = false;
    
    if (error > 0) {
      if (this->current_exposure_ < sc202cs_params::MAX_EXPOSURE) {
        this->current_exposure_ += sc202cs_params::AE_EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ < sc202cs_params::MAX_GAIN_INDEX) {
        this->current_gain_index_ += sc202cs_params::AE_GAIN_STEP;
        changed = true;
      }
    } else {
      if (this->current_exposure_ > sc202cs_params::MIN_EXPOSURE) {
        this->current_exposure_ -= sc202cs_params::AE_EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ > sc202cs_params::MIN_GAIN_INDEX) {
        this->current_gain_index_ -= sc202cs_params::AE_GAIN_STEP;
        changed = true;
      }
    }
    
    if (changed && this->ae_command_queue_) {
      AECommand cmd = {this->current_exposure_, this->current_gain_index_};
      xQueueSend(this->ae_command_queue_, &cmd, 0);
    }
  }
}

uint32_t MipiDsiCam::calculate_brightness_() {
  if (!this->display_buffer_) {
    return 128;
  }
  
  uint32_t sum = 0;
  uint32_t count = 0;
  
  uint32_t center_offset = (this->height_ / 2) * this->width_ * 2 + (this->width_ / 2) * 2;
  
  for (int i = 0; i < 50; i++) {
    uint32_t offset = center_offset + (i * 400);
    if (offset + 1 < this->frame_buffer_size_) {
      uint16_t pixel = (this->display_buffer_[offset + 1] << 8) | this->display_buffer_[offset];
      
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;
      
      sum += (r * 8 * 299 + g * 4 * 587 + b * 8 * 114) / 1000;
      count++;
    }
  }
  
  return count > 0 ? (sum / count) : 128;
}

void MipiDsiCam::loop() {
  if (this->streaming_) {
    this->update_auto_exposure_();
    
    uint32_t now = millis();
    if (now - this->last_frame_log_time_ >= 3000) {
      float fps = this->total_frames_received_ / 3.0f;
      
      ESP_LOGI(TAG, "📸 FPS: %.1f | frames: %u | exp:0x%04X gain:%u | esp_video", 
               fps, this->frame_number_, 
               this->current_exposure_, this->current_gain_index_);
      
      this->total_frames_received_ = 0;
      this->last_frame_log_time_ = now;
    }
  }
}

void MipiDsiCam::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI Camera (esp_video pipeline):");
  if (this->sensor_driver_) {
    ESP_LOGCONFIG(TAG, "  Sensor: %s", this->sensor_driver_->get_name());
  }
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Pipeline: esp_video (official Espressif)");
  ESP_LOGCONFIG(TAG, "  Auto Exposure: %s", 
                this->auto_exposure_enabled_ ? "ON" : "OFF");
}

// Méthodes de contrôle
void MipiDsiCam::set_auto_exposure(bool enabled) {
  this->auto_exposure_enabled_ = enabled;
  ESP_LOGI(TAG, "Auto Exposure: %s", enabled ? "ON" : "OFF");
}

void MipiDsiCam::set_ae_target_brightness(uint8_t target) {
  this->ae_target_brightness_ = target;
  ESP_LOGI(TAG, "AE target: %u", target);
}

void MipiDsiCam::set_manual_exposure(uint16_t exposure) {
  this->current_exposure_ = exposure;
  if (this->sensor_driver_ && this->ae_command_queue_) {
    AECommand cmd = {exposure, this->current_gain_index_};
    xQueueSend(this->ae_command_queue_, &cmd, 0);
    ESP_LOGI(TAG, "Manual exposure: 0x%04X", exposure);
  }
}

void MipiDsiCam::set_manual_gain(uint8_t gain) {
  this->current_gain_index_ = gain;
  if (this->sensor_driver_ && this->ae_command_queue_) {
    AECommand cmd = {this->current_exposure_, gain};
    xQueueSend(this->ae_command_queue_, &cmd, 0);
    ESP_LOGI(TAG, "Manual gain: %u", gain);
  }
}

void MipiDsiCam::adjust_exposure(uint16_t exposure) {
  set_manual_exposure(exposure);
}

void MipiDsiCam::adjust_gain(uint8_t gain) {
  set_manual_gain(gain);
}

void MipiDsiCam::set_brightness_level(uint8_t level) {
  if (level > 10) level = 10;
  uint16_t exposure = 0x400 + (level * 0x0B0);
  uint8_t gain = level * 6;
  ESP_LOGI(TAG, "🔆 Brightness level %u: exp=0x%04X gain=%u", level, exposure, gain);
  set_manual_exposure(exposure);
  vTaskDelay(pdMS_TO_TICKS(50));
  set_manual_gain(gain);
}

}  // namespace mipi_dsi_cam
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
