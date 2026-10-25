#include "Phase/PhaseStateStore.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <unistd.h>

namespace {

// phase_name()の逆引き用。全PhaseIDを総当たりして一致するものを探す。
bool try_parse_phase_id(const std::string& name, PhaseID& out)
{
    static const std::array<PhaseID, 10> kAllPhases{
        PhaseID::Standby,       PhaseID::Store,          PhaseID::Eject,
        PhaseID::LandDetection, PhaseID::Deployment,     PhaseID::Takeoff,
        PhaseID::GPSNavigate,   PhaseID::VisionNavigate, PhaseID::Land,
        PhaseID::Terminate,
    };
    for (const PhaseID id : kAllPhases)
    {
        if (name == phase_name(id))
        {
            out = id;
            return true;
        }
    }
    return false;
}

} // namespace

PhaseStateStore::PhaseStateStore(std::string path) : path_(std::move(path))
{
    // 初回起動時、保存先ディレクトリ(例: state/)が無くても落ちないように作っておく。
    const std::filesystem::path p(path_);
    if (p.has_parent_path())
    {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec)
        {
            std::cerr << "[PhaseStateStore] WARNING: failed to create directory "
                      << p.parent_path() << ": " << ec.message() << "\n";
        }
    }
}

void PhaseStateStore::save(PhaseID id) const
{
    const std::string tmp_path = path_ + ".tmp";

    // tmpファイルへ書いてfsyncしてからrenameすることで、書き込み途中の電源瞬断で
    // path_の中身が壊れる(中途半端な文字列になる)ことを防ぐ。renameは同一
    // ファイルシステム内であればatomicなので、renameの前後どちらかの内容にしかならない。
    FILE* fp = std::fopen(tmp_path.c_str(), "w");
    if (!fp)
    {
        std::cerr << "[PhaseStateStore] WARNING: failed to open " << tmp_path << " for write\n";
        return;
    }
    const std::string content = std::string(phase_name(id)) + "\n";
    const size_t written = std::fwrite(content.data(), 1, content.size(), fp);
    std::fflush(fp);
    ::fsync(fileno(fp));
    std::fclose(fp);

    if (written != content.size())
    {
        std::cerr << "[PhaseStateStore] WARNING: short write to " << tmp_path << "\n";
        return;
    }

    if (std::rename(tmp_path.c_str(), path_.c_str()) != 0)
    {
        std::cerr << "[PhaseStateStore] WARNING: failed to rename " << tmp_path << " to " << path_ << "\n";
    }
}

bool PhaseStateStore::load(PhaseID& out) const
{
    std::ifstream ifs(path_);
    if (!ifs)
    {
        return false; // 未保存(初回起動など)。呼び出し側でfresh_start_idにフォールバックする。
    }

    std::string line;
    if (!std::getline(ifs, line))
    {
        return false;
    }
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
    {
        line.pop_back();
    }

    if (!try_parse_phase_id(line, out))
    {
        std::cerr << "[PhaseStateStore] WARNING: unrecognized phase name in " << path_
                  << ": \"" << line << "\"\n";
        return false;
    }
    return true;
}

PhaseID PhaseStateStore::load_resume_phase(PhaseID fresh_start_id) const
{
    PhaseID persisted;
    if (!load(persisted))
    {
        return fresh_start_id;
    }

    switch (persisted)
    {
    // Store/Eject/LandDetectionで中断した場合: これらは基本的に「まだ判定中」の
    // フェーズで、副作用の大きいアクチュエータ操作(展開・離陸等)より前なので、
    // 中断した地点そのものから再開して問題ない。
    case PhaseID::Store:
    case PhaseID::Eject:
    case PhaseID::LandDetection:
        std::cout << "[PhaseStateStore] Resuming from " << phase_name(persisted)
                  << " (persisted phase before restart)\n";
        return persisted;

    // Deployment以降(離陸・航法・着陸)で中断した場合: 機体姿勢やGPS/EKF状態など
    // 複雑な前提を積んだフェーズなので、途中から素朴に再開するのは危険。
    // 一段階戻ってLandDetectionからやり直し、着地判定を再度確定させてから
    // Deployment以降を安全に進める。
    case PhaseID::Deployment:
    case PhaseID::Takeoff:
    case PhaseID::GPSNavigate:
    case PhaseID::VisionNavigate:
    case PhaseID::Land:
        std::cout << "[PhaseStateStore] Power interruption detected during "
                  << phase_name(persisted) << ". Restarting from LandDetection.\n";
        return PhaseID::LandDetection;

    // Standby: まだミッション開始前。Terminate: 前回ミッションは完了済み。
    // どちらも通常のミッション開始点から始めればよい。
    case PhaseID::Standby:
    case PhaseID::Terminate:
    default:
        return fresh_start_id;
    }
}
