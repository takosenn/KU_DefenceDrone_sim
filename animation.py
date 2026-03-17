import numpy as np
from connect_Coppelia import Simulation
from parameter import Params
from various_calculation import Various
from csv_save import set_csv_header, save_csv_data, set_error_csv_header
from datetime import datetime

"""現実時間の日本時間"""
Japan_time = datetime.now()

"""CSVのヘッダーを設定（CSV保存が有効な場合のみ）"""
if Params["save_csv"]:
    set_csv_header(Japan_time, "ro_i")
    set_csv_header(Japan_time, "theta")
    set_csv_header(Japan_time, "alpha_i")
    set_csv_header(Japan_time, "alpha_i_minus")
    set_csv_header(Japan_time, "omega_i")
    set_csv_header(Japan_time, "eta")
    set_csv_header(Japan_time, "e_i_1")
    set_csv_header(Japan_time, "e_i_2")
    set_csv_header(Japan_time, "fi")
    set_error_csv_header(Japan_time)  # 誤差データ用のヘッダーを追加


class Animation:
    def __init__(self):
        """アニメーション制御クラスの初期化"""
        # シミュレーション関連オブジェクト
        self.sim = Simulation()
        self.various = Various()

        # 状態フラグの初期化
        self._initialize_flags()

        # ターゲット関連変数の初期化
        self._initialize_target_variables()

        # エージェント位置追跡配列の初期化
        self._initialize_position_arrays()

        # エージェント角度・速度配列の初期化
        self._initialize_angular_velocity_arrays()

        # エージェント距離配列の初期化
        self._initialize_distance_arrays()

        # 制御・誤差配列の初期化
        self._initialize_control_arrays()

        # インデックスマッピングの初期化
        self._initialize_index_mapping()

    def _initialize_flags(self):
        """状態フラグの初期化"""
        self.initialized = False  # CoppeliaSim座標による初期化フラグ
        self.mapping_initialized = False  # theta順マッピングの初期化フラグ
        self.last_sort_step = -1  # 最後にソートを実行したステップ番号

    def _initialize_target_variables(self):
        """ターゲット関連変数の初期化"""
        self.target_position = [0, 0, 0]
        self.prev_target_position = [0, 0, 0]
        self.target_initial_z = 0  # z座標の初期値（後で設定）
        self.target_theta = 0  # ターゲットの絶対角度
        self.target_velocity = np.array([0.0, 0.0])  # 2D速度ベクトル

    def _initialize_position_arrays(self):
        """エージェント位置追跡配列の初期化（ワールド座標・ローカル座標）"""
        n = Params["num_agents"]

        # CoppeliaSim から取得した実際の位置（角速度などの計算用）
        self.agent_positions_from_sim = [[0, 0, 0] for _ in range(n)]
        self.prev_agent_positions_from_sim = [[0, 0, 0] for _ in range(n)]

        # 制御入力で計算・更新する位置（CoppeliaSim への送信用）
        self.current_world_agent_positions = [[0, 0, 0] for _ in range(n)]
        self.prev_world_agent_positions = [[0, 0, 0] for _ in range(n)]
        self.prev_prev_world_agent_positions = [[0, 0, 0] for _ in range(n)]

        # ローカル座標系の位置（ターゲットからの相対位置）
        self.local_agent_positions = [[0, 0, 0] for _ in range(n)]
        self.prev_local_agent_positions = [[0, 0, 0] for _ in range(n)]

        # 速度配列
        self.current_world_agent_velocities = [[0, 0, 0] for _ in range(n)]

    def _initialize_angular_velocity_arrays(self):
        """エージェント角度・角速度配列の初期化"""
        n = Params["num_agents"]

        self.theta = [0.0] * n  # 方位角
        self.prev_theta = [0.0] * n

        self.omega_i = [0.0] * n  # 平滑化された角速度（制御用）
        self.omega_i_raw = [0.0] * n  # 生の角速度（グラフプロット用）
        self.omega_i_plus = [0.0] * n  # 隣接エージェント（+1）の角速度
        self.omega_i_minus = [0.0] * n  # 隣接エージェント（-1）の角速度
        self.omega_i_history = [[] for _ in range(n)]  # 角速度履歴（平滑化用）

        self.alpha_i = [0.0] * n  # 角距離（隣接エージェント間）
        self.alpha_i_minus = [0.0] * n

    def _initialize_distance_arrays(self):
        """エージェント距離配列の初期化"""
        n = Params["num_agents"]

        self.ro_i = [0.0] * n  # ターゲットからの相対距離
        self.prev_ro_i = [0.0] * n
        self.eta = [0.0] * n  # 距離の時間微分

    def _initialize_control_arrays(self):
        """制御・誤差配列の初期化"""
        n = Params["num_agents"]

        self.e_i_1 = [0.0] * n  # 誤差項1（距離誤差）
        self.e_i_2 = [0.0] * n  # 誤差項2（角速度誤差）
        self.fi = [0.0] * n  # 制御パラメータfi

    def _initialize_index_mapping(self):
        """インデックスマッピングの初期化（theta順のエージェント割り当て）"""
        self.agent_index_mapping = list(range(Params["num_agents"]))

    def animate(
        self,
        i,
        target_position_from_sim,
        agent_positions_from_sim,
        formation_type="circle",
    ):
        """
        メインアニメーション処理

        Args:
            i: 現在のステップ番号
            target_position_from_sim: CoppeliaSimから取得したターゲット位置
            agent_positions_from_sim: CoppeliaSimから取得したエージェント位置
            formation_type: "semicircle" or "circle" - フォーメーションタイプ
        """
        current_time = i * Params["frame_time"]

        # 1. 初回初期化
        if not self.initialized:
            self._initialize_from_sim(
                target_position_from_sim, agent_positions_from_sim
            )

        # 2. ターゲット状態の更新
        self._update_target_state(target_position_from_sim)

        # 3. エージェント位置の更新
        self._update_agent_positions_from_sim(agent_positions_from_sim)

        # 4. 全エージェントの状態計算（theta, ro_i, eta, omega）
        agent_velocities_3d = self._calculate_agent_states()

        # 5. theta順のエージェントマッピング更新
        self._update_agent_mapping(i, formation_type)

        # 6. 角距離の計算
        self._calculate_angular_distances()

        # 7. 制御入力計算と位置更新
        self._calculate_control_inputs_and_update_positions(i, agent_velocities_3d)

        # 8. CSV保存
        if Params["save_csv"]:
            self._save_csv_data(current_time)

        # 9. CoppeliaSim同期
        self._sync_to_coppelia()

    def _initialize_from_sim(self, target_position_from_sim, agent_positions_from_sim):
        """CoppeliaSim から取得した実座標で初期化"""
        print("CoppeliaSim から取得した実座標で初期化中...")

        # ターゲットの初期化
        self.target_initial_z = target_position_from_sim[2]
        self.target_position = list(target_position_from_sim)
        self.prev_target_position = self.target_position.copy()

        # 各エージェントの初期化
        for j in range(Params["num_agents"]):
            # 位置の初期化
            self.agent_positions_from_sim[j] = list(agent_positions_from_sim[j])
            self.prev_agent_positions_from_sim[j] = list(agent_positions_from_sim[j])
            self.current_world_agent_positions[j] = list(agent_positions_from_sim[j])
            self.prev_world_agent_positions[j] = list(agent_positions_from_sim[j])
            self.prev_prev_world_agent_positions[j] = list(agent_positions_from_sim[j])

            # ローカル座標と距離の初期化
            local_pos = np.array(agent_positions_from_sim[j]) - np.array(
                target_position_from_sim
            )
            self.local_agent_positions[j] = local_pos
            self.prev_local_agent_positions[j] = local_pos.copy()
            distance = self.various.Distance(local_pos)
            self.ro_i[j] = distance
            self.prev_ro_i[j] = distance

            # 角度の初期化
            self.theta[j] = self.various.Theta(local_pos)
            self.prev_theta[j] = self.theta[j]

            print(
                f"  Agent[{j}]: 初期距離 ro_i = {distance:.3f} m, 初期角度 theta = {self.theta[j]:.3f} rad"
            )

        self.initialized = True
        print("初期化完了\n")

    def _update_target_state(self, target_position_from_sim):
        """ターゲットの位置と速度を更新"""
        self.target_position = list(target_position_from_sim)

        # CoppeliaSim から直接速度を取得
        target_velocity_3d, _ = self.sim.get_Drone_Velocity()
        self.target_velocity = np.array([target_velocity_3d[0], target_velocity_3d[1]])

    def _update_agent_positions_from_sim(self, agent_positions_from_sim):
        """CoppeliaSim から取得したエージェント位置を保存"""
        for j in range(Params["num_agents"]):
            self.agent_positions_from_sim[j] = list(agent_positions_from_sim[j])
            self.current_world_agent_positions[j] = list(agent_positions_from_sim[j])

    def _calculate_agent_states(self):
        """全エージェントの状態を計算（theta, ro_i, eta, omega）"""
        _, agent_velocities_3d = self.sim.get_Drone_Velocity()

        for j in range(Params["num_agents"]):
            # ローカル座標系の位置・距離・角度
            self.local_agent_positions[j] = np.array(
                self.agent_positions_from_sim[j]
            ) - np.array(self.target_position)
            self.ro_i[j] = self.various.Distance(self.local_agent_positions[j])
            self.theta[j] = self.various.Theta(self.local_agent_positions[j])
            self.prev_theta[j] = self.various.Theta(self.prev_local_agent_positions[j])

            # 距離の時間微分（eta）
            self.eta[j] = (self.ro_i[j] - self.prev_ro_i[j]) / Params["frame_time"]

            # 角速度の計算と平滑化
            omega_raw = self.various.Angular_velocity(self.theta[j], self.prev_theta[j])
            self.omega_i_raw[j] = omega_raw
            self.omega_i_history[j].append(omega_raw)
            if len(self.omega_i_history[j]) > Params["omega_history_size"]:
                self.omega_i_history[j].pop(0)
            self.omega_i[j] = np.mean(self.omega_i_history[j])

        return agent_velocities_3d

    def _update_agent_mapping(self, step, formation_type):
        """theta順でエージェントをソートしてマッピングを更新"""
        # フォーメーションタイプに応じた設定を選択
        if formation_type == "semicircle":
            theta_sort_enabled = Params["theta_sort_enabled_semicircle"]
            sort_interval = Params["theta_sort_interval_semicircle"]
        else:  # circle
            theta_sort_enabled = Params["theta_sort_enabled_circle"]
            sort_interval = Params["theta_sort_interval_circle"]

        # ソートが必要かチェック
        should_sort = theta_sort_enabled and (
            not self.mapping_initialized
            or (step - self.last_sort_step) >= sort_interval
        )

        if should_sort:
            # theta値でソート
            theta_with_index = [(j, self.theta[j]) for j in range(Params["num_agents"])]
            theta_with_index.sort(key=lambda x: x[1])
            self.agent_index_mapping = [agent_idx for agent_idx, _ in theta_with_index]

            self.mapping_initialized = True
            self.last_sort_step = step

    def _calculate_angular_distances(self):
        """角距離を計算（theta順に基づいて隣接関係を決定）"""
        for logical_j in range(Params["num_agents"]):
            actual_j = self.agent_index_mapping[logical_j]
            logical_j_plus = (logical_j + 1) % Params["num_agents"]
            logical_j_minus = (logical_j - 1) % Params["num_agents"]
            actual_j_plus = self.agent_index_mapping[logical_j_plus]
            actual_j_minus = self.agent_index_mapping[logical_j_minus]

            alpha_raw, alpha_minus_raw = self.various.Angular_distance(
                self.theta[actual_j],
                self.theta[actual_j_plus],
                self.theta[actual_j_minus],
            )
            self.alpha_i[actual_j] = alpha_raw
            self.alpha_i_minus[actual_j] = alpha_minus_raw

    def _calculate_control_inputs_and_update_positions(self, step, agent_velocities_3d):
        """制御入力を計算してエージェント位置を更新"""
        for logical_j in range(Params["num_agents"]):
            # マッピングから実際のインデックスを取得
            actual_j = self.agent_index_mapping[logical_j]
            actual_j_plus = self.agent_index_mapping[
                (logical_j + 1) % Params["num_agents"]
            ]
            actual_j_minus = self.agent_index_mapping[
                (logical_j - 1) % Params["num_agents"]
            ]

            # 隣接エージェントの角速度を設定
            self.omega_i_plus[actual_j] = np.copy(self.omega_i[actual_j_plus])
            self.omega_i_minus[actual_j] = np.copy(self.omega_i[actual_j_minus])

            # 制御入力を計算
            (
                u_r,
                u_theta,
                self.e_i_1[actual_j],
                self.e_i_2[actual_j],
                self.fi[actual_j],
            ) = self.various.caluculate(
                step,
                actual_j,
                self.alpha_i[actual_j],
                self.alpha_i_minus[actual_j],
                self.omega_i_plus[actual_j],
                self.omega_i[actual_j],
                self.omega_i_minus[actual_j],
                self.ro_i[actual_j],
                self.eta[actual_j],
                logical_j,
            )

            # 速度と位置を更新
            self._update_agent_velocity_and_position(actual_j, u_r, u_theta)

            # 前回値を更新
            self._update_previous_values(actual_j)

        self.prev_target_position = np.copy(self.target_position)

    def _update_agent_velocity_and_position(self, agent_idx, u_r, u_theta):
        """エージェントの速度と位置を更新"""
        # ローカル座標からワールド座標へ変換
        u_world_2d = self.various.coordinate_trans(
            self.theta[agent_idx], [u_r, u_theta]
        )
        u_world = np.append(u_world_2d, 0)

        # 速度を更新（速度制限を適用）
        new_velocity = u_world * Params["frame_time"] + np.array(
            self.current_world_agent_velocities[agent_idx]
        )
        velocity_magnitude = np.linalg.norm(new_velocity)
        if velocity_magnitude > Params["agent_speed_max"]:
            new_velocity = new_velocity / velocity_magnitude * Params["agent_speed_max"]
        self.current_world_agent_velocities[agent_idx] = new_velocity.tolist()

        # 位置を更新
        updated_position = (
            np.array(self.current_world_agent_positions[agent_idx])
            + new_velocity * Params["frame_time"]
        )
        self.current_world_agent_positions[agent_idx] = updated_position.tolist()

    def _update_previous_values(self, agent_idx):
        """エージェントの前回値を更新"""
        self.prev_ro_i[agent_idx] = np.copy(self.ro_i[agent_idx])
        self.prev_theta[agent_idx] = np.copy(self.theta[agent_idx])
        self.prev_local_agent_positions[agent_idx] = np.copy(
            self.local_agent_positions[agent_idx]
        )
        self.prev_prev_world_agent_positions[agent_idx] = np.copy(
            self.prev_world_agent_positions[agent_idx]
        )
        self.prev_world_agent_positions[agent_idx] = np.copy(
            self.current_world_agent_positions[agent_idx]
        )
        self.prev_agent_positions_from_sim[agent_idx] = np.copy(
            self.agent_positions_from_sim[agent_idx]
        )

    def _save_csv_data(self, current_time):
        """データをCSVに保存（論理順でソート）"""

        # 論理順（theta順）でデータを並び替え
        def to_logical_order(data):
            return [
                data[self.agent_index_mapping[j]] for j in range(Params["num_agents"])
            ]

        data_dict = {
            "ro_i": to_logical_order(self.ro_i),
            "alpha_i": to_logical_order(self.alpha_i),
            "alpha_i_minus": to_logical_order(self.alpha_i_minus),
            "omega_i": to_logical_order(self.omega_i_raw),  # 生の角速度を保存
            "eta": to_logical_order(self.eta),
            "e_i_1": to_logical_order(self.e_i_1),
            "e_i_2": to_logical_order(self.e_i_2),
            "theta": to_logical_order(self.theta),
            "fi": to_logical_order(self.fi),
        }

        for name, data in data_dict.items():
            save_csv_data(Japan_time, current_time, data, name)

    def _sync_to_coppelia(self):
        """計算結果をCoppeliaSim に同期"""
        self.sim.setAgentposition(self.current_world_agent_positions)
