#pragma once

#include <string>

#include "Phase/Phase.hpp"

// 電源瞬断（Raspiの予期しない再起動を伴う想定）からの復帰用に、現在のフェーズを
// ファイルへ永続化するクラス。
//
// - save()はフェーズ遷移のたびに呼ぶ想定。書き込みはtmpファイル→fsync→renameの
//   atomic writeパターンで行うため、書き込み途中で電源が落ちても保存先ファイルが
//   壊れた内容で上書きされることはない（renameは丸ごと成功/丸ごと失敗のどちらか）。
// - load_resume_phase()は起動直後に一度だけ呼ぶ想定。永続化されたフェーズから
//   「再開すべきフェーズ」を判断して返す。判断ルールは実装(.cpp)のコメント参照。
//
// 注意: 別ミッション（別の日の別フライト）を新規に始める前は、保存ファイル
// （デフォルトでは state/phase_state.txt）を削除してから起動すること。
// このクラスは「同一ミッション中の電源瞬断からの復帰」のみを想定しており、
// 前回ミッションの残骸から誤って再開しないようにするのは運用側の責務。
class PhaseStateStore {
public:
    explicit PhaseStateStore(std::string path);

    // 現在のフェーズをファイルへ上書き保存する。
    void save(PhaseID id) const;

    // 永続化されたフェーズを読み、再開すべきフェーズを返す。
    // 保存が無い/読めない場合はfresh_start_idを返す。
    PhaseID load_resume_phase(PhaseID fresh_start_id) const;

private:
    // 保存されているフェーズを読む。読めなければfalseを返す。
    bool load(PhaseID& out) const;

    std::string path_;
};
