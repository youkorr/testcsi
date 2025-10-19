#include "mipi_dsi_cam.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include "mipi_dsi_cam_drivers_generated.h"
#include "sc202cs_params.h"
#include "software_wb.h"  // 🆕 White Balance software

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace esphome {
namespace mipi_dsi_cam {

static const char *const TAG = "mipi_dsi_cam";

void MipiDsiCam::setup() {
  ESP_LOGI(TAG, "Init MIPI Camera (low latency + WB software)");
  ESP_LOGI(TAG, "  Sensor type: %s", this->sensor_type_.c_str());
  
  // 🔥 Charger paramètres depuis sc202cs_params.h
  this->current_exposure_ = sc202cs_params::DEFAULT_EXPOSURE;
  this->current_gain_index_ = sc202cs_params::DEFAULT_GAIN_INDEX;
  this->ae_target_brightness_ = sc202cs_params::AE_TARGET_BRIGHTNESS;
  this->auto_exposure_enabled_ = sc202cs_params::AUTO_EXPOSURE_ENABLED;
  this->wb_red_gain_ = sc202cs_params::WB_RED_GAIN;
  this->wb_green_gain_ = sc202cs_params::WB_GREEN_GAIN;
  this->wb_blue_gain_ = sc202cs_params::WB_BLUE_GAIN;
  
  // 🆕 Créer l'objet WB software
  this->wb_processor_ = new SoftwareWhiteBalance(
    this->wb_red_gain_, 
    this->wb_green_gain_, 
    this->wb_blue_gain_
  );
  
  ESP_LOGI(TAG, "🔥 SC202CS Params:");
  ESP_LOGI(TAG, "   Exposure: 0x%04X (%d)", this->current_exposure_, this->current_exposure_);
  ESP_LOGI(TAG, "   Gain: %d", this->current_gain_index_);
  ESP_LOGI(TAG, "   WB Software: R=%.2f G=%.2f B=%.2f", 
           this->wb_red_gain_, this->wb_green_gain_, this->wb_blue_gain_);
  ESP_LOGI(TAG, "   AE: %s, target=%d", 
           this->auto_exposure_enabled_ ? "ON" : "OFF", this->ae_target_brightness_);
  
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
  } else {
    ESP_LOGI(TAG, "No external clock - sensor uses internal clock");
  }
  
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "LDO init failed");
    this->mark_failed();
    return;
  }
  
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "CSI init failed");
    this->mark_failed();
    return;
  }
  
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "ISP init failed");
    this->mark_failed();
    return;
  }
  
  if (!this->allocate_buffer_()) {
    ESP_LOGE(TAG, "Buffer alloc failed");
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
  
  // Démarrer la tâche AE asynchrone (priorité basse)
  xTaskCreatePinnedToCore(
    ae_task_,
    "ae_task",
    3072,
    this,
    1,  // Priorité basse pour ne pas perturber l'affichage
    &this->ae_task_handle_,
    0   // Core 0 (CSI/ISP sur core 1)
  );
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Camera ready (%ux%u) - Triple buffering + WB Software + Async AE", 
           this->width_, this->height_);
}

bool MipiDsiCam::create_sensor_driver_() {
  ESP_LOGI(TAG, "Creating driver for: %s", this->sensor_type_.c_str());
  
  this->sensor_driver_ = create_sensor_driver(this->sensor_type_, this);
  
  if (this->sensor_driver_ == nullptr) {
    ESP_LOGE(TAG, "Unknown or unavailable sensor: %s", this->sensor_type_.c_str());
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
  ESP_LOGI(TAG, "Sensor stabilized");
  
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

bool MipiDsiCam::init_ldo_() {
  ESP_LOGI(TAG, "Init LDO MIPI");
  
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "LDO OK (2.5V)");
  return true;
}

bool MipiDsiCam::init_csi_() {
  ESP_LOGI(TAG, "Init MIPI-CSI");
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = this->width_;
  csi_config.v_res = this->height_;
  csi_config.lane_bit_rate_mbps = this->lane_bitrate_mbps_;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = this->lane_count_;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 10;
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI failed: %d", ret);
    return false;
  }
  
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = MipiDsiCam::on_csi_new_frame_,
    .on_trans_finished = MipiDsiCam::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Callbacks failed: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Enable CSI failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "CSI OK");
  return true;
}

bool MipiDsiCam::init_isp_() {
  ESP_LOGI(TAG, "Init ISP");
  
  uint32_t isp_clock_hz = 120000000;
  
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = this->width_;
  isp_config.v_res = this->height_;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = isp_clock_hz;
  isp_config.bayer_order = (color_raw_element_order_t)this->bayer_pattern_;
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP creation failed: 0x%x", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP enable failed: 0x%x", ret);
    esp_isp_del_processor(this->isp_handle_);
    this->isp_handle_ = nullptr;
    return false;
  }
  
  this->configure_white_balance_();
  
  ESP_LOGI(TAG, "ISP OK");
  return true;
}

void MipiDsiCam::configure_white_balance_() {
  if (!this->isp_handle_) return;
  
  // 🆕 CORRECTION: Pour SC202CS et OV5647, on utilise WB software au lieu de matériel
  if (this->sensor_type_ == "ov5647" || this->sensor_type_ == "sc202cs") {
    ESP_LOGI(TAG, "%s détecté - AWB matériel désactivé → WB Software activé", 
             this->sensor_type_.c_str());
    ESP_LOGI(TAG, "   Utilisation WB logiciel: R=%.2f G=%.2f B=%.2f",
             this->wb_red_gain_, this->wb_green_gain_, this->wb_blue_gain_);
    
    // 🔥 Le WB sera appliqué dans capture_frame() via wb_processor_
    this->use_software_wb_ = true;
    return;
  }
  
  // Pour les autres sensors, tenter AWB matériel
  esp_isp_awb_config_t awb_config = {};
  awb_config.sample_point = ISP_AWB_SAMPLE_POINT_AFTER_CCM;
  awb_config.window.top_left.x = this->width_ / 4;
  awb_config.window.top_left.y = this->height_ / 4;
  awb_config.window.btm_right.x = (this->width_ * 3) / 4;
  awb_config.window.btm_right.y = (this->height_ * 3) / 4;
  
  esp_err_t ret = esp_isp_new_awb_controller(this->isp_handle_, &awb_config, &this->awb_ctlr_);
  
  if (ret == ESP_OK && this->awb_ctlr_ != nullptr) {
    esp_isp_awb_controller_enable(this->awb_ctlr_);
    ESP_LOGI(TAG, "✅ AWB matériel activé");
    this->use_software_wb_ = false;
  } else {
    ESP_LOGW(TAG, "AWB matériel non disponible (0x%x) → WB Software", ret);
    this->use_software_wb_ = true;
  }
}

bool MipiDsiCam::allocate_buffer_() {
  this->frame_buffer_size_ = this->width_ * this->height_ * 2;
  
  // Allouer 3 buffers au lieu de 2
  for (int i = 0; i < NUM_BUFFERS; i++) {
    this->frame_buffers_[i] = (uint8_t*)heap_caps_aligned_alloc(
      64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
    );
    
    if (!this->frame_buffers_[i]) {
      ESP_LOGE(TAG, "Buffer %d alloc failed", i);
      return false;
    }
  }
  
  this->display_buffer_ = this->frame_buffers_[2];  // Buffer initial d'affichage
  
  ESP_LOGI(TAG, "Buffers: 3x%u bytes (triple buffering)", this->frame_buffer_size_);
  return true;
}

// ISR optimisé avec triple buffering
bool IRAM_ATTR MipiDsiCam::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  MipiDsiCam *cam = (MipiDsiCam*)user_data;
  
  // Donner le buffer de capture au CSI
  uint8_t idx = cam->capture_buffer_index_.load(std::memory_order_relaxed);
  trans->buffer = cam->frame_buffers_[idx];
  trans->buflen = cam->frame_buffer_size_;
  
  return false;
}

// ISR optimisé - swap atomique des buffers
bool IRAM_ATTR MipiDsiCam::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  MipiDsiCam *cam = (MipiDsiCam*)user_data;
  
  if (trans->received_size > 0) {
    // Rotation atomique: capture -> ready, ready -> capture
    uint8_t old_capture = cam->capture_buffer_index_.load(std::memory_order_acquire);
    uint8_t old_ready = cam->ready_buffer_index_.load(std::memory_order_acquire);
    
    // Swap capture <-> ready (le 3ème buffer reste pour l'affichage)
    cam->ready_buffer_index_.store(old_capture, std::memory_order_release);
    cam->capture_buffer_index_.store(old_ready, std::memory_order_release);
    
    cam->display_buffer_ready_.store(true, std::memory_order_release);
    cam->frame_number_.fetch_add(1, std::memory_order_relaxed);
    cam->total_frames_received_++;
  }
  
  return false;
}

bool MipiDsiCam::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Start streaming");
  
  this->total_frames_received_ = 0;
  this->last_frame_log_time_ = millis();
  
  if (this->sensor_driver_) {
    esp_err_t ret = this->sensor_driver_->start_stream();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Sensor start failed: %d", ret);
      return false;
    }
    delay(100);
  }
  
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI start failed: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming active");
  return true;
}

bool MipiDsiCam::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_cam_ctlr_stop(this->csi_handle_);
  
  if (this->sensor_driver_) {
    this->sensor_driver_->stop_stream();
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "Streaming stopped");
  return true;
}

// capture_frame optimisé - swap atomique avec le display buffer + WB software
bool MipiDsiCam::capture_frame() {
  if (!this->streaming_) {
    return false;
  }
  
  // Vérifier s'il y a une nouvelle frame
  if (!this->display_buffer_ready_.load(std::memory_order_acquire)) {
    return false;
  }
  
  // Swap atomique: ready -> display
  uint8_t ready_idx = this->ready_buffer_index_.load(std::memory_order_acquire);
  this->display_buffer_ = this->frame_buffers_[ready_idx];
  this->display_buffer_ready_.store(false, std::memory_order_release);
  
  // 🆕 CORRECTION: Appliquer WB software si activé
  if (this->use_software_wb_ && this->wb_processor_ && this->display_buffer_) {
    // Appliquer WB seulement toutes les 2 frames pour économiser du CPU
    // Si vous voulez encore plus de FPS, changez à % 3 ou % 4
    if (this->frame_number_.load() % 2 == 0) {
      this->wb_processor_->apply_to_buffer(
        this->display_buffer_, 
        this->width_, 
        this->height_
      );
    }
  }
  
  return true;
}

// Tâche AE asynchrone - ne bloque plus la loop principale
void MipiDsiCam::ae_task_(void* param) {
  MipiDsiCam* cam = (MipiDsiCam*)param;
  AECommand cmd;
  
  ESP_LOGI(TAG, "AE task started");
  
  while (true) {
    // Attendre une commande de la loop principale
    if (xQueueReceive(cam->ae_command_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
      
      // Exécuter les écritures I2C (lentes) ici
      if (cam->sensor_driver_) {
        cam->sensor_driver_->set_exposure(cmd.exposure);
        vTaskDelay(pdMS_TO_TICKS(5));  // Laisser le temps au I2C
        cam->sensor_driver_->set_gain(cmd.gain);
      }
      
      // Petit délai pour éviter de spammer le I2C
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
      // Image trop sombre
      if (this->current_exposure_ < sc202cs_params::MAX_EXPOSURE) {
        this->current_exposure_ += sc202cs_params::AE_EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ < sc202cs_params::MAX_GAIN_INDEX) {
        this->current_gain_index_ += sc202cs_params::AE_GAIN_STEP;
        changed = true;
      }
    } else {
      // Image trop lumineuse
      if (this->current_exposure_ > sc202cs_params::MIN_EXPOSURE) {
        this->current_exposure_ -= sc202cs_params::AE_EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ > sc202cs_params::MIN_GAIN_INDEX) {
        this->current_gain_index_ -= sc202cs_params::AE_GAIN_STEP;
        changed = true;
      }
    }
    
    // Envoyer commande à la tâche AE asynchrone au lieu de bloquer
    if (changed && this->ae_command_queue_) {
      AECommand cmd = {this->current_exposure_, this->current_gain_index_};
      xQueueSend(this->ae_command_queue_, &cmd, 0);  // Non-bloquant
    }
  }
}

uint32_t MipiDsiCam::calculate_brightness_() {
  if (!this->display_buffer_) {
    return 128;
  }
  
  uint32_t sum = 0;
  uint32_t count = 0;
  
  // Échantillonner au centre
  uint32_t center_offset = (this->height_ / 2) * this->width_ * 2 + (this->width_ / 2) * 2;
  
  for (int i = 0; i < 50; i++) {  // Réduit de 100 à 50 échantillons
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
    // AE non-bloquante
    this->update_auto_exposure_();
    
    // Stats toutes les 3s
    uint32_t now = millis();
    if (now - this->last_frame_log_time_ >= 3000) {
      float fps = this->total_frames_received_ / 3.0f;
      
      ESP_LOGI(TAG, "📸 FPS: %.1f | frames: %u | exp:0x%04X gain:%u | WB:%s", 
               fps, this->frame_number_.load(), 
               this->current_exposure_, this->current_gain_index_,
               this->use_software_wb_ ? "SW" : "HW");
      
      this->total_frames_received_ = 0;
      this->last_frame_log_time_ = now;
    }
  }
}

void MipiDsiCam::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI Camera (Low Latency + WB Software):");
  if (this->sensor_driver_) {
    ESP_LOGCONFIG(TAG, "  Sensor: %s", this->sensor_driver_->get_name());
  }
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Buffering: Triple (3 buffers)");
  ESP_LOGCONFIG(TAG, "  Auto Exposure: %s (async)", 
                this->auto_exposure_enabled_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  White Balance: %s", 
                this->use_software_wb_ ? "Software" : "Hardware");
  ESP_LOGCONFIG(TAG, "  Config: sc202cs_params.h");
}

// Méthodes publiques de contrôle
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

void MipiDsiCam::set_white_balance_gains(float r, float g, float b) {
  this->wb_red_gain_ = r;
  this->wb_green_gain_ = g;
  this->wb_blue_gain_ = b;
  
  // 🆕 Mettre à jour l'objet WB processor
  if (this->wb_processor_) {
    this->wb_processor_->set_gains(r, g, b);
  }
  
  ESP_LOGI(TAG, "🎨 WB gains updated: R=%.2f G=%.2f B=%.2f", r, g, b);
}

}  // namespace mipi_dsi_cam
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
