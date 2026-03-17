import numpy as np
from typing import Any

Params: dict[str, Any] = {
    # --- システム設定 ---
    "use_mocap": False,  # True: モーションキャプチャ, False: CoppeliaSim
    "save_csv": True,  # True: CSV保存する, False: CSV保存しない

    # --- エージェント設定 ---
    "num_agents": 6,  # Agentの数[台]

    # --- ターゲット設定 ---
    "target_move": True,  # True: targetを移動させる, False: targetを静止させる
    "target_move_speed": 0.1,  # targetの目標位置への移動速度[m/s]
    "target_goal_x": -10,  # targetの目標位置x座標[m]
    "target_goal_y": -10,  # targetの目標位置y座標[m]
    "target_tolerance": 0.1,  # target到達判定の許容誤差[m]

    # --- シミュレーション設定 ---
    "frames": 2400,  # アニメーションのフレーム数[frame]
    "frame_time": 0.05,  # 1フレームにかかる時間[s]

    # --- 制御パラメータ ---
    "agent_speed_max": 6,  # エージェントの最大速度[m/s]
    "agent_speed_max_values": [8.5,7],  # 円形フォーメーション時の各エージェントの最大速度[m/s]
    "omega_history_size": 1,  # エージェントの角速度履歴サイズ
    "R": 4,  # targetとAgentの理想の距離(フォーメーションの半径)[m]
    "Omega": 0.5,  # Agentの理想角速度[rad/s]
    "omega_values": [0.2,0.15], # 各エージェントの角速度[rad/s]
    "l1": 6,
    "l2": 2.5,  # 初期値（l2_valuesから動的に更新される）
    "l2_values": [1.5, 1.5],  # l2の切り替え値リスト

    # --- ソート設定 ---
    # 半円フォーメーション用（移行段階）
    "theta_sort_enabled_semicircle": True,  # True: theta順ソートを有効化, False: 無効化（固定マッピング）
    "theta_sort_interval_semicircle": 200,  # theta順ソートの更新間隔[ステップ]

    # 円形フォーメーション用
    "theta_sort_enabled_circle": False,  # True: theta順ソートを有効化, False: 無効化（固定マッピング）
    "theta_sort_interval_circle": 200,  # theta順ソートの更新間隔[ステップ]
}

# 計算で求まるパラメータ
# 各エージェントの理想相対角距離[rad]（デフォルトは均等分配）
Params["d_i"] = [
    np.pi / 3,
    np.pi / 3,
    np.pi / 3,
    np.pi / 3,
    np.pi / 3,
    np.pi / 3,
]  # 6台の場合
Params["fps"] = 1 / Params["frame_time"]  # 1秒間に更新するフレーム数[frame]


def update_params(new_params):
    """
    外部からパラメータを更新する関数

    Args:
        new_params (dict): 更新するパラメータの辞書
    """
    for key, value in new_params.items():
        if key in Params:
            Params[key] = value
            print(f"パラメータ更新: {key} = {value}")
        else:
            print(f"警告: 未知のパラメータ '{key}' は無視されました")

    # frame_time が変更された場合は fps も更新
    if "frame_time" in new_params:
        Params["fps"] = 1 / Params["frame_time"]
        print(f"パラメータ自動更新: fps = {Params['fps']}")
