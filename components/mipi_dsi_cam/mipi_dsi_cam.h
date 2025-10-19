#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <string>
#include <atomic>

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "esp_ldo_regulator.h"
}
#endif

namespace esphome {
namespace mipi_dsi_cam {

enum PixelFormat {
  PIXEL_FORMAT_RGB565 = 0,
  PIXEL_FORMAT_YUV422 = 1,
  PIXEL_FORMAT_RAW8 = 2,
};

class ISensorDriver {
public:
  virtual ~ISensorDriver() = default;
  
  virtual const char* get_name() const = 0;
  virtual uint16_t get_pid() const = 0;
  virtual uint8_t get_i2c_address() const = 0;
  virtual uint8_t get_lane_count() const = 0;
  virtual uint8_t get_bayer_pattern() const = 0;
  virtual uint16_t get_lane_bitrate_mbps() const = 0;
  virtual uint16_t get_width() const = 0;
  virtual uint16_t get_height() const = 0;
  virtual uint8_t get_fps() const = 0;
  
  virtual esp_err_t init() = 0;
  virtual esp_err_t read_id(uint16_t* pid) = 0;
  virtual esp_err_t start_stream() = 0;
  virtual esp_err_t stop_stream() = 0;
  virtual esp_err_t set_gain(uint32_t gain_index) = 0;
  virtual esp_err_t set_exposure(uint32_t exposure) = 0;
  virtual esp_err_t write_register(uint16_t reg, uint8_t value) = 0;
  virtual esp_err_t read_register(uint16_t reg, uint8_t* value) = 0;
};

class MipiDsiCam : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(int8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_sensor_type(const std::string &type) { this->sensor_type_ = type; }
  void set_sensor_address(uint8_t addr) { this->sensor_address_ = addr; }
  void set_lane_count(uint8_t lanes) { this->lane_count_ = lanes; }
  void set_bayer_pattern(uint8_t pattern) { this->bayer_pattern_ = pattern; }
  void set_lane_bitrate(uint16_t mbps) { this->lane_bitrate_mbps_ = mbps; }
  void set_resolution(uint16_t w, uint16_t h) { this->width_ = w; this->height_ = h; }
  void set_pixel_format(PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }

  // 🔧 API modifiée pour gérer la latence
  bool capture_frame();
  bool has_new_frame() const { return this->display_buffer_ready_.load(); }
  uint32_t get_frame_number() const { return this->frame_number_.load(); }
  
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_; }
  
  uint8_t* get_image_data() { return this->display_buffer_; }
  size_t get_image_size() const { return this->frame_buffer_size_; }
  uint16_t get_image_width() const { return this->width_; }
  uint16_t get_image_height() const { return this->height_; }
  
  bool has_external_clock() const { return this->external_clock_pin_ >= 0; }

  // Auto Exposure et White Balance
  void set_auto_exposure(bool enabled);
  void set_ae_target_brightness(uint8_t target);
  void set_manual_exposure(uint16_t exposure);
  void set_manual_gain(uint8_t gain_index);
  void set_white_balance_gains(float red, float green, float blue);
  void adjust_exposure(uint16_t exposure_value);
  void adjust_gain(uint8_t gain_index);
  void set_brightness_level(uint8_t level);

 protected:
  int8_t external_clock_pin_{-1};
  uint32_t external_clock_frequency_{24000000};
  GPIOPin *reset_pin_{nullptr};
  
  std::string sensor_type_{""};
  uint8_t sensor_address_{0x36};
  uint8_t lane_count_{1};
  uint8_t bayer_pattern_{0};
  uint16_t lane_bitrate_mbps_{576};
  uint16_t width_{1280};
  uint16_t height_{720};
  
  std::string name_{"MIPI Camera"};
  PixelFormat pixel_format_{PIXEL_FORMAT_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};

  bool initialized_{false};
  bool streaming_{false};
  
  // 🆕 Triple buffering avec atomics pour éviter les race conditions
  static constexpr size_t NUM_BUFFERS = 3;
  uint8_t *frame_buffers_[NUM_BUFFERS]{nullptr, nullptr, nullptr};
  std::atomic<uint8_t> capture_buffer_index_{0};  // Buffer CSI écrit dedans
  std::atomic<uint8_t> ready_buffer_index_{1};    // Buffer prêt à afficher
  uint8_t *display_buffer_{nullptr};              // Buffer affiché
  
  std::atomic<bool> display_buffer_ready_{false};
  std::atomic<uint32_t> frame_number_{0};
  
  size_t frame_buffer_size_{0};
  
  uint32_t total_frames_received_{0};
  uint32_t last_frame_log_time_{0};
  
  ISensorDriver *sensor_driver_{nullptr};

  // Auto Exposure - déporté dans une tâche séparée
  bool auto_exposure_enabled_{false};
  uint16_t current_exposure_{0x4d};
  uint8_t current_gain_index_{0};
  uint32_t ae_target_brightness_{80};
  uint32_t last_ae_update_{0};
  
  // 🆕 File pour les commandes AE asynchrones
  struct AECommand {
    uint16_t exposure;
    uint8_t gain;
  };
  QueueHandle_t ae_command_queue_{nullptr};
  TaskHandle_t ae_task_handle_{nullptr};
  
  // White Balance correction
  float wb_red_gain_{1.9f};
  float wb_green_gain_{0.1f};
  float wb_blue_gain_{1.3f};
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  esp_cam_ctlr_handle_t csi_handle_{nullptr};
  isp_proc_handle_t isp_handle_{nullptr};
  esp_ldo_channel_handle_t ldo_handle_{nullptr};
  isp_awb_ctlr_t awb_ctlr_{nullptr};
  
  bool create_sensor_driver_();
  bool init_sensor_();
  bool init_external_clock_();
  bool init_ldo_();
  bool init_csi_();
  bool init_isp_();
  bool allocate_buffer_();
  
  void configure_white_balance_();
  void update_auto_exposure_();
  uint32_t calculate_brightness_();
  
  // 🆕 Tâche AE asynchrone
  static void ae_task_(void* param);
  
  static bool IRAM_ATTR on_csi_new_frame_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
  
  static bool IRAM_ATTR on_csi_frame_done_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
#endif
  void apply_software_ccm_(uint8_t* buffer, size_t size);
};

}  // namespace mipi_dsi_cam
}  // namespace esphome
