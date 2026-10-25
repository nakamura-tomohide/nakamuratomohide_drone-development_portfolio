#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

// TFmini-S（Benewake）用のI2Cドライバ。
// RP2040（光センサ、addr 0x08）と同じI2Cバスを共有し、addr 0x10で読む。
// UARTモードと同じ9バイトフレーム(0x59 0x59ヘッダ+距離+強度+温度+チェックサム)を
// レジスタ選択なしでそのまま読み出す（TFmini-S公式のI2C読み出し方式）。
// 参考実装: https://github.com/budryerson/TFMini-Plus-I2C
struct LidarReading {
    float distance_m{0.0f};
    bool valid{false};
};

// 傾き補正: TFMiniBridgeは機体直下(body -Z)を向いている前提の取り付けなので、
// 機体が傾くとビームが鉛直から逸れ、実際の鉛直対地高度より長い斜め距離を返す
// （鉛直成分 = 斜め距離 × cos(roll) × cos(pitch)。roll/pitch複合の傾き角の余弦）。
// 傾きが小さいうちは、この補正による改善よりroll/pitch推定自体の誤差の方が
// 支配的になりうるため、kLidarTiltCorrectionThresholdDegを超えた場合のみ補正する。
constexpr float kLidarTiltCorrectionThresholdDeg = 22.5f;

struct TiltCorrectedDistance {
    float distance_m{0.0f};
    bool  corrected{false}; // 傾き補正を適用したか（閾値を超えた場合のみtrue）
};

// slant_distance_m: LiDARが実際に測った（機体が傾いていれば斜めの）距離。
// roll_deg/pitch_degは機体姿勢（オイラー角、TelemetryData準拠）。
TiltCorrectedDistance apply_lidar_tilt_correction(float slant_distance_m, float roll_deg, float pitch_deg);

class TFMiniBridge {
public:
    static constexpr unsigned kDefaultI2CBus = 1;
    static constexpr unsigned kDefaultI2CAddress = 0x10;

    // use_hakoniwa=trueの場合、I2Cの代わりにhakoniwa_lidar_bridge.pyが書き出す
    // hakoniwa_agl_pathを読んで距離とする（箱庭シミュレータ用。実機I2C初期化は行わない）。
    explicit TFMiniBridge(unsigned i2c_bus = kDefaultI2CBus,
                          unsigned i2c_address = kDefaultI2CAddress,
                          bool use_hakoniwa = false,
                          std::string hakoniwa_agl_path = "/tmp/hakoniwa_lidar_agl.txt");
    ~TFMiniBridge();

    bool connect();
    void disconnect();
    bool is_connected() const;

    // 1回だけ読む（ブロッキング）。
    bool read_distance(float& distance_m);

    // デバッグ用: ヘッダ/チェックサム検証なしで生の9バイトフレームをそのまま返す。
    bool debug_read_raw(uint8_t* buffer, std::size_t length);

    LidarReading get_latest_reading() const;

    // get_latest_reading().validは「直近1回の読み取り」だけを見た瞬間的な値で、
    // 紐・虫・埃・反射などによる一発だけの誤りと、断線・センサ故障のような
    // 継続的な異常を区別できない。is_reliable()は直近の連続無効読み取り回数を見て、
    // kConsecutiveInvalidToDistrust回以上連続で無効が続いた場合にfalseを返す
    // （＝「LiDARが明らかに有効な値を返していない」と判断できる状態）。
    // 呼び出し側は「get_latest_reading().valid && is_reliable()」の両方を見て、
    // 片方でも欠ければPX4側の高度（気圧/GPS）にフォールバックすることを想定している。
    bool is_reliable() const;

    void start_polling();
    void stop_polling();
    bool is_polling() const;

private:
    void polling_loop();
    bool read_frame(uint8_t* buffer, std::size_t length);
    bool read_distance_from_hakoniwa(float& distance_m);
    // 直近の有効読み取り(distance_history_)から中央値フィルタ後の距離を計算する。
    // latest_mutex_を呼び出し側で保持済みの前提（polling_loop()専用の内部ヘルパー）。
    float push_and_compute_median_locked(float new_distance_m);

private:
    static constexpr int kPollingPeriodMs = 50; // 20Hz
    static constexpr int kMaxRetries = 3;
    static constexpr int kRetryDelayMs = 10;

    static constexpr std::size_t kFrameLength = 9;
    static constexpr uint8_t kHeaderByte = 0x59;
    static constexpr uint16_t kMinValidStrength = 100;
    static constexpr uint16_t kInvalidDistanceCm = 65535;
    // 20cm未満はTFmini-Sの近距離ブラインドゾーンまたは旋回時の逸脱とみなし、無効として扱う。
    static constexpr uint16_t kMinValidDistanceCm = 20;

    // 連続でこの回数以上無効読み取りが続いたら「明らかに壊れている/測れていない」と
    // みなす。kPollingPeriodMs=50msなので10回=500ms程度。
    static constexpr int kConsecutiveInvalidToDistrust = 10;

    // 無効読み取りがこの回数以内（kPollingPeriodMs=50msなので3回=150ms程度）なら、
    // 機体の一瞬の傾き・床の継ぎ目通過などによる瞬間的な取りこぼしとみなし、
    // 直前の有効値をそのまま保持して呼び出し側にはvalid=trueを返し続ける。
    // これを超えたら実際にvalid=falseへ切り替える
    // （kConsecutiveInvalidToDistrustはさらにその先の「明らかに壊れている」判定）。
    static constexpr int kHoldMissesBeforeInvalid = 3;

    // 中央値フィルタのウィンドウ幅。kPollingPeriodMs=50msなので5サンプル=250ms相当。
    // 紐・虫・埃・反射等による単発の異常値(フレーム自体は正常でvalidになる値)を
    // 弾くための対策。2026-09-12の飛行で、GPSNavigate::goal_to_NED()が単発の
    // 生lidar値をそのままアンカーに使い、その値がノイズで2m以上ズレていたために
    // 目標高度全体がズレる事故が確認されたため導入。奇数にして中央値の定義を明確にする。
    static constexpr std::size_t kMedianFilterSize = 5;

    // 箱庭連携用: hakoniwa_lidar_bridge.pyが書き出すファイルの更新時刻(mtime)が
    // これより古ければ「ブリッジが更新を止めている」とみなして無効扱いにする。
    // ブリッジ側は50ms間隔で書く想定なので、その10倍程度の余裕を持たせてある。
    static constexpr int kMaxStaleMs = 500;

    unsigned i2c_bus_;
    unsigned i2c_address_;
    int i2c_handle_{-1};
    bool pigpio_initialized_{false};

    bool use_hakoniwa_{false};
    std::string hakoniwa_agl_path_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::atomic<int>  consecutive_invalid_count_{0};

    std::thread polling_thread_;
    mutable std::mutex io_mutex_;
    mutable std::mutex latest_mutex_;

    LidarReading latest_reading_{};

    // 中央値フィルタ用のリングバッファ(latest_mutex_で保護。polling_loop()内でのみ書く)。
    // distance_history_len_はまだ埋まっていない起動直後のバッファサイズを表す
    // （kMedianFilterSize件溜まるまではその時点までの件数で中央値を取る）。
    std::array<float, kMedianFilterSize> distance_history_{};
    std::size_t distance_history_len_{0};
    std::size_t distance_history_next_{0};
};
