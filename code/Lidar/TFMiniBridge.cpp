#include "TFMiniBridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#ifndef __APPLE__
#include <pigpio.h>
#endif

TiltCorrectedDistance apply_lidar_tilt_correction(float slant_distance_m, float roll_deg, float pitch_deg) {
    const float roll_rad  = roll_deg  * static_cast<float>(M_PI) / 180.0f;
    const float pitch_rad = pitch_deg * static_cast<float>(M_PI) / 180.0f;
    // 鉛直成分の比率。roll/pitchそれぞれの傾きの余弦を掛け合わせたもの
    // （機体の下向きボディZ軸が世界座標の鉛直から見てどれだけ立っているか）。
    const float cos_tilt = std::cos(roll_rad) * std::cos(pitch_rad);
    // cos_tiltから合成の傾き角[deg]を逆算する。roll/pitchが±90度を超えることは
    // 通常運用では想定しないため、acosの定義域(-1〜1)に収まる前提だが、
    // 念のためclampしておく。
    const float tilt_deg = std::acos(std::clamp(cos_tilt, -1.0f, 1.0f)) * 180.0f / static_cast<float>(M_PI);

    if (tilt_deg > kLidarTiltCorrectionThresholdDeg) {
        return TiltCorrectedDistance{slant_distance_m * cos_tilt, true};
    }
    return TiltCorrectedDistance{slant_distance_m, false};
}

TFMiniBridge::TFMiniBridge(unsigned i2c_bus, unsigned i2c_address,
                           bool use_hakoniwa, std::string hakoniwa_agl_path)
    : i2c_bus_(i2c_bus), i2c_address_(i2c_address),
      use_hakoniwa_(use_hakoniwa), hakoniwa_agl_path_(std::move(hakoniwa_agl_path)) {}

TFMiniBridge::~TFMiniBridge() {
    stop_polling();
    disconnect();
}

#ifdef __APPLE__
// Mac開発用スタブ。実機(Raspberry Pi + pigpio)無しでビルド・実行できるようにするためのダミー実装。
// use_hakoniwa_=trueの場合はこのスタブがそのまま箱庭LiDARブリッジ読み込みの土台になる
// （read_distance()側で分岐するので、ここは元々のスタブのままでよい）。
bool TFMiniBridge::connect() {
    connected_ = true;
    return true;
}

void TFMiniBridge::disconnect() {
    stop_polling();
    connected_ = false;
}

bool TFMiniBridge::read_frame(uint8_t*, std::size_t) {
    return false;
}

#else
// Raspberry Pi + pigpio 実機用実装。
bool TFMiniBridge::connect() {
    if (connected_) {
        return true;
    }

    // 箱庭シミュレータ連携時はI2C/pigpioを一切触らない
    // （hakoniwa_lidar_bridge.pyが書き出すファイルをread_distance()側で読むだけ）。
    if (use_hakoniwa_) {
        connected_ = true;
        return true;
    }

    if (!pigpio_initialized_) {
        if (gpioInitialise() < 0) {
            return false;
        }
        // pigpioがgpioInitialise()内でSIGHUPハンドラを独自に登録し、nohup由来の
        // 無視設定を上書きしてしまうため、SIGHUPだけ無視に戻す（SSH切断でプロセスが
        // 落ちるのを防ぐ）。他のシグナル（SIGTERM/SIGSEGV等）へのpigpioの後始末は残す。
        std::signal(SIGHUP, SIG_IGN);
        pigpio_initialized_ = true;
    }

    i2c_handle_ = i2cOpen(i2c_bus_, i2c_address_, 0);
    if (i2c_handle_ < 0) {
        gpioTerminate();
        pigpio_initialized_ = false;
        return false;
    }

    connected_ = true;
    return true;
}

void TFMiniBridge::disconnect() {
    stop_polling();

    if (i2c_handle_ >= 0) {
        i2cClose(i2c_handle_);
        i2c_handle_ = -1;
    }

    connected_ = false;

    if (pigpio_initialized_) {
        gpioTerminate();
        pigpio_initialized_ = false;
    }
}

// Benewake公式マニュアル(SJ-PM-TFmini-S A00, Table 10/12)準拠の読み取り手順。
// I2Cモードでは、単に読むだけ／レジスタ書き込み+リピートスタートで読むだけでは
// 有効なフレームが返らない（実機で確認済み: どちらも0x59 0x59ヘッダが来ず0埋め）。
// 公式のタイミングシーケンス(Table 10)は
//   Start Addr[W] コマンド Stop → (待機) → Start Addr[R] 9バイト Stop
// と、書き込みと読み取りが「別トランザクション」であることを明示しており、
// 送るコマンドも任意の1バイトではなく "Obtain Data Frame" コマンド
// 5A 05 00 01 60 (Head, Len, ID, Data, Checksum=先頭4バイトの和の下位8bit) を
// 使う必要がある（Table 12）。
bool TFMiniBridge::read_frame(uint8_t* buffer, std::size_t length) {
    if (!connected_ || buffer == nullptr || length == 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(io_mutex_);

    // "Obtain Data Frame"（cm単位・9バイト）コマンド。マニュアルTable 12より。
    // i2cWriteDevice()がchar*（非const）を要求するためconstにはできない。
    static uint8_t kObtainFrameCmd[5] = {0x5A, 0x05, 0x00, 0x01, 0x60};

    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        const int write_result = i2cWriteDevice(
            i2c_handle_, reinterpret_cast<char*>(kObtainFrameCmd),
            sizeof(kObtainFrameCmd));

        if (write_result == 0) {
            // マニュアル§5.6: コマンド後の読み取りは1ms程度待ってから、が推奨。
            // 別トランザクションにする必要がある（STOPを挟む）ため、
            // ここでread_frame()を抜けずそのまま読み取りに進む。
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            std::vector<char> read_buffer(length, 0);
            const int read_count =
                i2cReadDevice(i2c_handle_, read_buffer.data(), static_cast<unsigned>(length));

            if (read_count == static_cast<int>(length)) {
                std::memcpy(buffer, read_buffer.data(), length);
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
    }

    return false;
}
#endif

bool TFMiniBridge::is_connected() const {
    return connected_;
}

bool TFMiniBridge::debug_read_raw(uint8_t* buffer, std::size_t length) {
    return read_frame(buffer, length);
}

// hakoniwa_lidar_bridge.pyが機体直下の点群から算出したAGL距離[m]を書き出しているファイルを
// 読む。ブリッジ側は「直下の円錐内に有効な点が無かった」場合に負値(-1)を書き込む取り決め
// なので、それをそのまま「無効」として扱う。
//
// 加えて、ファイルの更新時刻(mtime)による鮮度チェックを行う。hakoniwa_lidar_bridge.py
// プロセスが落ちる/起動し忘れる等で更新が止まっても、ファイル自体は最後に書かれた値の
// ままファイルシステム上に残り続けるため、mtimeを見ないと「同じ古い値をずっと有効な
// 最新値として読み続ける」状態を検知できない（実際にこれで高度が固定されたまま
// 気づかずに飛行させてしまう事故が起きた）。ブリッジは0.05s間隔で書き込む想定なので、
// それより十分緩い閾値（kMaxStaleMs）を超えて更新が無ければ無効として扱う。
bool TFMiniBridge::read_distance_from_hakoniwa(float& distance_m) {
    std::error_code ec;
    const auto mtime = std::filesystem::last_write_time(hakoniwa_agl_path_, ec);
    if (ec) {
        return false; // ファイルが存在しない等
    }
    const auto age = std::filesystem::file_time_type::clock::now() - mtime;
    if (age > std::chrono::milliseconds(kMaxStaleMs)) {
        return false; // 更新が止まっている（ブリッジ未起動・クラッシュ等）
    }

    std::ifstream ifs(hakoniwa_agl_path_);
    if (!ifs) {
        return false;
    }

    float value = 0.0f;
    ifs >> value;
    // 0.2m未満は箱庭シミュレータの近距離ブラインドゾーンまたは旋回時の逸脱とみなし、無効として扱う。
    //9/3の航続距離試験のデバッグで追加
    if (!ifs || value < 0.2f) {
        return false;
    }

    distance_m = value;
    return true;
}

bool TFMiniBridge::read_distance(float& distance_m) {
    if (use_hakoniwa_) {
        return read_distance_from_hakoniwa(distance_m);
    }

    uint8_t frame[kFrameLength] = {0};

    if (!read_frame(frame, kFrameLength)) {
        return false;
    }

    // ヘッダ検証。ここが合わなければプロトコルの前提が崩れているので即無効扱いにする。
    if (frame[0] != kHeaderByte || frame[1] != kHeaderByte) {
        return false;
    }

    // チェックサム検証（先頭8バイトの和の下位8bit）。
    uint8_t checksum = 0;
    for (std::size_t i = 0; i < kFrameLength - 1; ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    if (checksum != frame[kFrameLength - 1]) {
        return false;
    }

    const uint16_t distance_cm = static_cast<uint16_t>(frame[2]) | (static_cast<uint16_t>(frame[3]) << 8);
    const uint16_t strength    = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);

    // 信号強度不足・測定不能・20cm未満（ブラインドゾーン・逸脱）は無効として扱う。
    //9/3の航続距離試験のデバッグで追加
    if (strength < kMinValidStrength || distance_cm == kInvalidDistanceCm || distance_cm < kMinValidDistanceCm) {
        return false;
    }

    distance_m = static_cast<float>(distance_cm) / 100.0f;
    return true;
}

LidarReading TFMiniBridge::get_latest_reading() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return latest_reading_;
}

// 新しい有効読み取りをリングバッファに積み、その時点の中央値を返す。
// 呼び出し側(polling_loop())がlatest_mutex_を保持している前提。
// 紐・虫・埃・反射等でフレーム自体は正常(validなまま)に単発だけ異常な距離を
// 返すケースは、フレームレベルの無効判定(read_distance_from_frame内)では
// 弾けない。中央値は直近kMedianFilterSize件のうち過半数が同意しない限り
// 動かないため、こうした単発の外れ値を吸収できる。
float TFMiniBridge::push_and_compute_median_locked(float new_distance_m) {
    distance_history_[distance_history_next_] = new_distance_m;
    distance_history_next_ = (distance_history_next_ + 1) % kMedianFilterSize;
    if (distance_history_len_ < kMedianFilterSize) {
        ++distance_history_len_;
    }

    std::array<float, kMedianFilterSize> sorted = distance_history_;
    const std::size_t n = distance_history_len_;
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(n));
    return sorted[n / 2];
}

bool TFMiniBridge::is_reliable() const {
    return consecutive_invalid_count_.load() < kConsecutiveInvalidToDistrust;
}

void TFMiniBridge::start_polling() {
    if (running_ || !connected_) {
        return;
    }

    running_ = true;
    polling_thread_ = std::thread(&TFMiniBridge::polling_loop, this);
}

void TFMiniBridge::stop_polling() {
    running_ = false;

    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }
}

bool TFMiniBridge::is_polling() const {
    return running_;
}

void TFMiniBridge::polling_loop() {
    while (running_) {
        float distance_m = 0.0f;
        const bool ok = read_distance(distance_m);

        // 連続無効カウンタの更新。有効読み取りが1回でもあればすぐ回復させる
        // （壊れているかどうかの判断は「連続して無効」であることに意味があるため）。
        if (ok) {
            consecutive_invalid_count_.store(0);
        } else if (consecutive_invalid_count_.load() < kConsecutiveInvalidToDistrust) {
            ++consecutive_invalid_count_;
        }

        {
            std::lock_guard<std::mutex> lock(latest_mutex_);
            if (ok) {
                // 生値をそのまま採用せず、直近数サンプルの中央値を採用する。
                // フレーム自体は正常(validなまま)に単発だけ異常な距離を返すケース
                // （紐・虫・埃・反射等）は、フレームレベルの無効判定だけでは弾けない
                // ため。詳細はpush_and_compute_median_locked()のコメント参照。
                latest_reading_.distance_m = push_and_compute_median_locked(distance_m);
                // 長時間ドロップアウト直後(distance_history_len_==0からの復帰)は、
                // 中央値フィルタのサンプル数が1件しかなく、中央値の定義上その1発の
                // 生値がそのまま素通りしてしまう＝フィルタが実質無効化される。
                // kMedianFilterSize件(=250ms)分の連続した有効読み取りで履歴が
                // 埋まるまではvalidを立てず、直前の値を保持し続けることで、復帰直後の
                // 単発の外れ値が無防備に採用されるのを防ぐ（2026-09-14 Black Rock飛行:
                // 復帰直後の外れ値がそのまま高度制御に使われ、暴走上昇した事故の一因）。
                latest_reading_.valid = (distance_history_len_ >= kMedianFilterSize);
            } else if (consecutive_invalid_count_.load() <= kHoldMissesBeforeInvalid) {
                // 短時間の取りこぼし（機体の一瞬の傾き・床の継ぎ目通過等）とみなし、
                // 直前の有効値をそのまま保持する（distance_m/validどちらも更新しない）。
            } else {
                latest_reading_.valid = false;
                // 長時間ドロップアウトしていた間の古いサンプルを、復帰後の中央値に
                // 混ぜないようリセットする（復帰直後は新しい値だけで中央値を作り直す）。
                distance_history_len_ = 0;
                distance_history_next_ = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kPollingPeriodMs));
    }
}
