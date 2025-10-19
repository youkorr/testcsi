SENSOR_INFO = {
    'name': 'ov5647_vga',
    'manufacturer': 'OmniVision',
    'pid': 0x5647,
    'i2c_address': 0x36,
    'lane_count': 2,
    'bayer_pattern': 1,  # BGGR
    'lane_bitrate_mbps': 360,
    'width': 640,
    'height': 480,
    'fps': 60,  # HAUTE VITESSE !
}

REGISTERS = {
    'sensor_id_h': 0x300a,
    'sensor_id_l': 0x300b,
    'stream_mode': 0x0100,
    'software_reset': 0x0103,
    'gain_h': 0x350a,
    'gain_l': 0x350b,
    'exposure_h': 0x3500,
    'exposure_m': 0x3501,
    'exposure_l': 0x3502,
    'pll_multiplier': 0x3036,
    'pll_ctrl': 0x3037,
}

INIT_SEQUENCE = [
    # === PHASE 1: Reset et config de base (IDENTIQUE à tab5_camera) ===
    (0x0103, 0x01, 10),  # Software reset - WAIT 10ms (CRITIQUE!)
    (0x0100, 0x00, 0),   # Sleep mode OFF
    (0x3035, 0x41, 0),   # System clock config
    (0x303c, 0x11, 0),   # PLLS control
    
    # === PHASE 2: Configuration 800x480@60fps ===
    (0x3034, 0x18, 0),   # RAW8 mode (OV5647_8BIT_MODE)
    (0x3036, 0x80, 0),   # PLL multiplier: 128 (pour 100MHz IDI clock)
    (0x3106, 0xf5, 0),
    (0x3821, 0x03, 0),   # Horizontal binning
    (0x3820, 0x41, 0),   # Vertical binning
    (0x3827, 0xec, 0),
    (0x370c, 0x0f, 0),
    (0x3612, 0x59, 0),
    (0x3618, 0x00, 0),
    
    # ISP control
    (0x5000, 0xff, 0),   # Enable tous les ISP blocks
    (0x583e, 0xf0, 0),   # LSC max gain
    (0x583f, 0x20, 0),   # LSC min gain
    (0x5002, 0x41, 0),
    (0x5003, 0x08, 0),
    (0x5a00, 0x08, 0),
    
    # System control
    (0x3000, 0x00, 0),
    (0x3001, 0x00, 0),
    (0x3002, 0x00, 0),
    (0x3016, 0x08, 0),
    (0x3017, 0xe0, 0),
    (0x3018, 0x44, 0),
    (0x301c, 0xf8, 0),
    (0x301d, 0xf0, 0),
    (0x3a18, 0x00, 0),
    (0x3a19, 0xf8, 0),
    (0x3c01, 0x80, 0),
    (0x3c00, 0x40, 0),
    (0x3b07, 0x0c, 0),
    
    # Timing - HTS/VTS pour 60fps
    (0x380c, 0x07, 0),   # HTS H: 1896 pixels
    (0x380d, 0x68, 0),   # HTS L
    (0x380e, 0x02, 0),   # VTS H: 738 lines (pour 60fps au lieu de 984 pour 50fps)
    (0x380f, 0xe2, 0),   # VTS L
    
    # Subsample
    (0x3814, 0x31, 0),   # Horizontal odd subsample
    (0x3815, 0x31, 0),   # Vertical odd subsample
    (0x3708, 0x64, 0),
    (0x3709, 0x52, 0),
    
    # Window - X start: 500 (identique à 640p)
    (0x3800, 0x01, 0),   # X start H
    (0x3801, 0xf4, 0),   # X start L (500)
    
    # Window - Y start: 180 (centré pour 480 lignes)
    (0x3802, 0x00, 0),   # Y start H
    (0x3803, 0xb4, 0),   # Y start L (180 = 0xB4)
    
    # Window - X end: 2623 (identique à 640p)
    (0x3804, 0x0a, 0),   # X end H
    (0x3805, 0x3f, 0),   # X end L (2623)
    
    # Window - Y end: 1613 (180 + 480*3 - 1 pour binning 3:1)
    (0x3806, 0x06, 0),   # Y end H
    (0x3807, 0x4d, 0),   # Y end L (1613 = 0x64D)
    
    # Output size: 800x480
    (0x3808, 0x03, 0),   # Width H (800 = 0x320)
    (0x3809, 0x20, 0),   # Width L
    (0x380a, 0x01, 0),   # Height H (480 = 0x1E0)
    (0x380b, 0xe0, 0),   # Height L
    
    # Offset
    (0x3810, 0x00, 0),   # H offset H
    (0x3811, 0x08, 0),   # H offset L
    (0x3812, 0x00, 0),   # V offset H
    (0x3813, 0x00, 0),   # V offset L
    
    # Analog control
    (0x3630, 0x2e, 0),
    (0x3632, 0xe2, 0),
    (0x3633, 0x23, 0),
    (0x3634, 0x44, 0),
    (0x3636, 0x06, 0),
    (0x3620, 0x64, 0),
    (0x3621, 0xe0, 0),
    (0x3600, 0x37, 0),
    (0x3704, 0xa0, 0),
    (0x3703, 0x5a, 0),
    (0x3715, 0x78, 0),
    (0x3717, 0x01, 0),
    (0x3731, 0x02, 0),
    (0x370b, 0x60, 0),
    (0x3705, 0x1a, 0),
    
    # Frex control
    (0x3f05, 0x02, 0),
    (0x3f06, 0x10, 0),
    (0x3f01, 0x0a, 0),
    
    # AEC/AGC control
    (0x3a08, 0x01, 0),
    (0x3a09, 0x27, 0),
    (0x3a0a, 0x00, 0),
    (0x3a0b, 0xf6, 0),
    (0x3a0d, 0x04, 0),
    (0x3a0e, 0x03, 0),
    (0x3a0f, 0x58, 0),
    (0x3a10, 0x50, 0),
    (0x3a1b, 0x58, 0),
    (0x3a1e, 0x50, 0),
    (0x3a11, 0x60, 0),
    (0x3a1f, 0x28, 0),
    
    # BLC control
    (0x4001, 0x02, 0),
    (0x4004, 0x02, 0),
    (0x4000, 0x09, 0),
    
    # MIPI timing control
    (0x4837, 0x28, 0),   # MIPI PCLK period
    
    # Vertical and horizontal frame off
    (0x4050, 0x6e, 0),
    (0x4051, 0x8f, 0),
]

# Tables de gain
GAIN_VALUES = [
    1000, 1062, 1125, 1187, 1250, 1312, 1375, 1437,
    1500, 1562, 1625, 1687, 1750, 1812, 1875, 1937,
    2000, 2125, 2250, 2375, 2500, 2625, 2750, 2875,
    3000, 3125, 3250, 3375, 3500, 3625, 3750, 3875,
    4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750,
    6000, 6250, 6500, 6750, 7000, 7250, 7500, 7750,
    8000, 8500, 9000, 9500, 10000, 10500, 11000, 11500,
    12000, 12500, 13000, 13500, 14000, 14500, 15000, 15500,
]

GAIN_REGISTERS = [
    (0x00, 0x10), (0x00, 0x11), (0x00, 0x12), (0x00, 0x13),
    (0x00, 0x14), (0x00, 0x15), (0x00, 0x16), (0x00, 0x17),
    (0x00, 0x18), (0x00, 0x19), (0x00, 0x1a), (0x00, 0x1b),
    (0x00, 0x1c), (0x00, 0x1d), (0x00, 0x1e), (0x00, 0x1f),
    (0x00, 0x20), (0x00, 0x22), (0x00, 0x24), (0x00, 0x26),
    (0x00, 0x28), (0x00, 0x2a), (0x00, 0x2c), (0x00, 0x2e),
    (0x00, 0x30), (0x00, 0x32), (0x00, 0x34), (0x00, 0x36),
    (0x00, 0x38), (0x00, 0x3a), (0x00, 0x3c), (0x00, 0x3e),
    (0x00, 0x40), (0x00, 0x44), (0x00, 0x48), (0x00, 0x4c),
    (0x00, 0x50), (0x00, 0x54), (0x00, 0x58), (0x00, 0x5c),
    (0x00, 0x60), (0x00, 0x64), (0x00, 0x68), (0x00, 0x6c),
    (0x00, 0x70), (0x00, 0x74), (0x00, 0x78), (0x00, 0x7c),
    (0x00, 0x80), (0x00, 0x88), (0x00, 0x90), (0x00, 0x98),
    (0x00, 0xa0), (0x00, 0xa8), (0x00, 0xb0), (0x00, 0xb8),
    (0x00, 0xc0), (0x00, 0xc8), (0x00, 0xd0), (0x00, 0xd8),
    (0x00, 0xe0), (0x00, 0xe8), (0x00, 0xf0), (0x00, 0xf8),
]

def generate_driver_cpp():
    cpp_code = f'''
namespace esphome {{
namespace mipi_dsi_cam {{

namespace {SENSOR_INFO['name']}_regs {{
'''
    
    for name, addr in REGISTERS.items():
        cpp_code += f'    constexpr uint16_t {name.upper()} = 0x{addr:04X};\n'
    
    cpp_code += f'''
}}

struct {SENSOR_INFO['name'].upper()}InitRegister {{
    uint16_t addr;
    uint8_t value;
    uint16_t delay_ms;
}};

static const {SENSOR_INFO['name'].upper()}InitRegister {SENSOR_INFO['name']}_init_sequence[] = {{
'''
    
    for addr, value, delay in INIT_SEQUENCE:
        cpp_code += f'    {{0x{addr:04X}, 0x{value:02X}, {delay}}},\n'
    
    cpp_code += f'''
}};

struct {SENSOR_INFO['name'].upper()}GainRegisters {{
    uint8_t gain_h;
    uint8_t gain_l;
}};

static const {SENSOR_INFO['name'].upper()}GainRegisters {SENSOR_INFO['name']}_gain_map[] = {{
'''
    
    for gain_h, gain_l in GAIN_REGISTERS:
        cpp_code += f'    {{0x{gain_h:02X}, 0x{gain_l:02X}}},\n'
    
    cpp_code += f'''
}};

class {SENSOR_INFO['name'].upper()}Driver {{
public:
    {SENSOR_INFO['name'].upper()}Driver(esphome::i2c::I2CDevice* i2c) : i2c_(i2c) {{}}
    
    esp_err_t init() {{
        ESP_LOGI(TAG, "Init {SENSOR_INFO['name'].upper()} - 640x480@90fps ULTRA-RAPIDE");
        ESP_LOGI(TAG, "  PLL: 24MHz × 120 = 2880MHz → ~360 Mbps/lane");
        ESP_LOGI(TAG, "  Frame time: ~11ms (latence minimale)");
        
        // CRITIQUE : Respecter l'ordre exact - délai APRÈS écriture
        for (size_t i = 0; i < sizeof({SENSOR_INFO['name']}_init_sequence) / sizeof({SENSOR_INFO['name'].upper()}InitRegister); i++) {{
            const auto& reg = {SENSOR_INFO['name']}_init_sequence[i];
            
            // ✅ CORRECTION : Écrire le registre D'ABORD
            esp_err_t ret = write_register(reg.addr, reg.value);
            if (ret != ESP_OK) {{
                ESP_LOGE(TAG, "❌ Init failed at reg 0x%04X", reg.addr);
                return ret;
            }}
            
            // ✅ PUIS appliquer le délai APRÈS (comme dans tab5_camera)
            if (reg.delay_ms > 0) {{
                ESP_LOGD(TAG, "⏱️  Waiting %ums after reg 0x%04X", reg.delay_ms, reg.addr);
                vTaskDelay(pdMS_TO_TICKS(reg.delay_ms));
            }}
            
            // Log de progression
            if (i == 0) {{
                ESP_LOGI(TAG, "✓ Phase 1: Software reset done");
            }} else if (i == 4) {{
                ESP_LOGI(TAG, "✓ Phase 1: PLL configured for 90fps");
            }} else if (i == sizeof({SENSOR_INFO['name']}_init_sequence) / sizeof({SENSOR_INFO['name'].upper()}InitRegister) - 1) {{
                ESP_LOGI(TAG, "✓ Phase 2: VGA 90fps config done");
            }}
        }}
        
        ESP_LOGI(TAG, "✅ {SENSOR_INFO['name'].upper()} initialized - 90 FPS MODE ACTIVATED");
        return ESP_OK;
    }}
    
    esp_err_t read_id(uint16_t* pid) {{
        uint8_t pid_h, pid_l;
        
        ESP_LOGD(TAG, "Reading sensor ID...");
        
        esp_err_t ret = read_register({SENSOR_INFO['name']}_regs::SENSOR_ID_H, &pid_h);
        if (ret != ESP_OK) {{
            ESP_LOGE(TAG, "Failed to read PID H");
            return ret;
        }}
        
        ret = read_register({SENSOR_INFO['name']}_regs::SENSOR_ID_L, &pid_l);
        if (ret != ESP_OK) {{
            ESP_LOGE(TAG, "Failed to read PID L");
            return ret;
        }}
        
        *pid = (pid_h << 8) | pid_l;
        ESP_LOGI(TAG, "✓ Read PID: 0x%04X (H=0x%02X, L=0x%02X)", *pid, pid_h, pid_l);
        return ESP_OK;
    }}
    
    esp_err_t start_stream() {{
        ESP_LOGI(TAG, "🚀 Starting 90fps stream...");
        esp_err_t ret = write_register({SENSOR_INFO['name']}_regs::STREAM_MODE, 0x01);
        if (ret == ESP_OK) {{
            ESP_LOGI(TAG, "✓ Stream started - 90 FPS active!");
        }}
        return ret;
    }}
    
    esp_err_t stop_stream() {{
        ESP_LOGI(TAG, "Stopping stream...");
        return write_register({SENSOR_INFO['name']}_regs::STREAM_MODE, 0x00);
    }}
    
    esp_err_t set_gain(uint32_t gain_index) {{
        if (gain_index >= sizeof({SENSOR_INFO['name']}_gain_map) / sizeof({SENSOR_INFO['name'].upper()}GainRegisters)) {{
            gain_index = (sizeof({SENSOR_INFO['name']}_gain_map) / sizeof({SENSOR_INFO['name'].upper()}GainRegisters)) - 1;
        }}
        
        const auto& gain = {SENSOR_INFO['name']}_gain_map[gain_index];
        
        esp_err_t ret = write_register({SENSOR_INFO['name']}_regs::GAIN_H, gain.gain_h);
        if (ret != ESP_OK) return ret;
        
        ret = write_register({SENSOR_INFO['name']}_regs::GAIN_L, gain.gain_l);
        return ret;
    }}
    
    esp_err_t set_exposure(uint32_t exposure) {{
        uint8_t exp_h = (exposure >> 16) & 0xFF;
        uint8_t exp_m = (exposure >> 8) & 0xFF;
        uint8_t exp_l = exposure & 0xFF;
        
        esp_err_t ret = write_register({SENSOR_INFO['name']}_regs::EXPOSURE_H, exp_h);
        if (ret != ESP_OK) return ret;
        
        ret = write_register({SENSOR_INFO['name']}_regs::EXPOSURE_M, exp_m);
        if (ret != ESP_OK) return ret;
        
        ret = write_register({SENSOR_INFO['name']}_regs::EXPOSURE_L, exp_l);
        return ret;
    }}
    
    esp_err_t write_register(uint16_t reg, uint8_t value) {{
        uint8_t data[3] = {{
            static_cast<uint8_t>((reg >> 8) & 0xFF),
            static_cast<uint8_t>(reg & 0xFF),
            value
        }};
        
        auto err = i2c_->write_read(data, 3, nullptr, 0); // new write
        if (err != esphome::i2c::ERROR_OK) {{
            ESP_LOGE(TAG, "I2C write failed for reg 0x%04X", reg);
            return ESP_FAIL;
        }}
        return ESP_OK;
    }}
    
    esp_err_t read_register(uint16_t reg, uint8_t* value) {{
        uint8_t addr[2] = {{
            static_cast<uint8_t>((reg >> 8) & 0xFF),
            static_cast<uint8_t>(reg & 0xFF)
        }};
        
        auto err = i2c_->write_read(addr, 2, value, 1);
                                                       
                                                       
        if (err != esphome::i2c::ERROR_OK) {{
            ESP_LOGE(TAG, "I2C cmd failed for reg 0x%04X", reg);
            return ESP_FAIL;
        }}
        
        return ESP_OK;
    }}
    
private:
    esphome::i2c::I2CDevice* i2c_;
    static constexpr const char* TAG = "{SENSOR_INFO['name'].upper()}";
}};

class {SENSOR_INFO['name'].upper()}Adapter : public ISensorDriver {{
public:
    {SENSOR_INFO['name'].upper()}Adapter(i2c::I2CDevice* i2c) : driver_(i2c) {{}}
    
    const char* get_name() const override {{ return "OV5647 VGA (90fps)"; }}
    uint16_t get_pid() const override {{ return 0x{SENSOR_INFO['pid']:04X}; }}
    uint8_t get_i2c_address() const override {{ return 0x{SENSOR_INFO['i2c_address']:02X}; }}
    uint8_t get_lane_count() const override {{ return {SENSOR_INFO['lane_count']}; }}
    uint8_t get_bayer_pattern() const override {{ return {SENSOR_INFO['bayer_pattern']}; }}
    uint16_t get_lane_bitrate_mbps() const override {{ return {SENSOR_INFO['lane_bitrate_mbps']}; }}
    uint16_t get_width() const override {{ return {SENSOR_INFO['width']}; }}
    uint16_t get_height() const override {{ return {SENSOR_INFO['height']}; }}
    uint8_t get_fps() const override {{ return {SENSOR_INFO['fps']}; }}
    
    esp_err_t init() override {{ return driver_.init(); }}
    esp_err_t read_id(uint16_t* pid) override {{ return driver_.read_id(pid); }}
    esp_err_t start_stream() override {{ return driver_.start_stream(); }}
    esp_err_t stop_stream() override {{ return driver_.stop_stream(); }}
    esp_err_t set_gain(uint32_t gain_index) override {{ return driver_.set_gain(gain_index); }}
    esp_err_t set_exposure(uint32_t exposure) override {{ return driver_.set_exposure(exposure); }}
    esp_err_t write_register(uint16_t reg, uint8_t value) override {{ return driver_.write_register(reg, value); }}
    esp_err_t read_register(uint16_t reg, uint8_t* value) override {{ return driver_.read_register(reg, value); }}
    
private:
    {SENSOR_INFO['name'].upper()}Driver driver_;
}};

}}
}}
'''
    
    return cpp_code

def get_sensor_info():
    return SENSOR_INFO

def get_driver_code():
    return generate_driver_cpp()
