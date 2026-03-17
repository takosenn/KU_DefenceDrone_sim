# これを実行するとシミュレーションスタート

from parameter import Params
from animation import Animation
from patroll import Patroll
from connect_Coppelia import Simulation
from various_calculation import Various
from csv_save import plot_csv_data, save_error_csv_data
import numpy as np


class Main:
    def __init__(self):
        # シミュレーション関連オブジェクトの初期化
        self.sim = Simulation()
        self.ani = Animation()
        self.patroll = Patroll()
        self.various = Various()

        # Simulationインスタンスの共有
        self.ani.sim = self.sim
        self.patroll.sim = self.sim

        # ターゲットの目標位置と移動制御
        self.target_goal = np.array(
            [float(Params["target_goal_x"]), float(Params["target_goal_y"])]
        )
        self.target_position = None
        self.target_initial_z = None
        self.target_reached_goal = False
        self.target_tolerance = float(Params["target_tolerance"])

        # フォーメーション状態管理
        self.circle_formation_started = False
        self.formation_switch_counter = 0
        self.transitioning_to_circle = False
        self.transition_step_size = 0.01

    def run(self):
        try:
            # シミュレーションの初期化
            self._initialize_simulation()

            # シミュレーションループ
            i = 0
            current_sim_time = self.sim.get_simulation_time()
            target_simulation_time = Params["frames"] * Params["frame_time"]

            while current_sim_time < target_simulation_time:
                current_sim_time = self._execute_simulation_step(i)
                i += 1

        except KeyboardInterrupt:
            print("\nctrl+Cでシミュレーションが終了しました")
        except Exception as e:
            print(f"\nエラーが発生しました: {type(e).__name__}: {e}")
            print("シミュレーションを停止します...")
        finally:
            self._finalize_simulation()

    def _initialize_simulation(self):
        """シミュレーションの初期化処理"""
        self.sim.get_handles(Params["num_agents"])
        self.sim.start_simulation()
        self.sim.client.setStepping(True)

        # 初期位置を取得
        print("CoppeliaSim から初期位置を取得中...")
        initial_target_pos, initial_agent_pos = self.sim.get_Drone_position()
        print(f"Agent初期位置: {initial_agent_pos}")

        # ターゲットの初期化
        self._initialize_target(initial_target_pos)

        # シミュレーション開始メッセージ
        target_simulation_time = Params["frames"] * Params["frame_time"]
        print(
            f"同期モードでシミュレーション開始（目標時間: {target_simulation_time:.1f}秒）\n"
        )

    def _initialize_target(self, initial_target_pos):
        """ターゲットの初期化

        Args:
            initial_target_pos: CoppeliaSimから取得した初期位置
        """
        self.target_position = list(initial_target_pos)
        self.target_initial_z = initial_target_pos[2]
        print(
            f"Target: 初期位置 = [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]"
        )
        print(
            f"Target: 目標位置 = [{self.target_goal[0]:.3f}, {self.target_goal[1]:.3f}]\n"
        )

    def _execute_simulation_step(self, frame_index):
        """シミュレーションの1ステップを実行

        Args:
            frame_index: 現在のフレーム番号

        Returns:
            更新後のシミュレーション時間
        """
        # CoppeliaSim から現在の座標を取得
        target_position, agent_positions = self.sim.get_Drone_position()

        # 角度と角距離を計算
        alpha_i = self._compute_angular_distances(target_position, agent_positions)

        # 誤差データを保存
        self._save_error_data_if_enabled(frame_index)

        # ターゲットの位置を更新
        target_position = self._process_target_movement(target_position)

        # 相対距離を計算してフォーメーション制御
        min_ro_i, max_ro_i = self._compute_relative_distances(
            target_position, agent_positions
        )
        self._execute_formation_control(
            frame_index, target_position, agent_positions, min_ro_i, max_ro_i, alpha_i
        )

        # シミュレーションを1ステップ進める
        self.sim.step_simulation()

        return self.sim.get_simulation_time()

    def _compute_angular_distances(self, target_position, agent_positions):
        """全エージェントの角度と角距離を計算

        Args:
            target_position: ターゲット位置
            agent_positions: エージェントの位置リスト

        Returns:
            角距離のリスト
        """
        # 全エージェントの角度を計算
        theta = [
            self.various.Theta(np.array(agent_positions[j]) - np.array(target_position))
            for j in range(Params["num_agents"])
        ]

        # theta順にソートして角距離を計算
        sorted_theta = sorted(
            [(j, theta[j]) for j in range(Params["num_agents"])], key=lambda x: x[1]
        )

        # 各エージェントの角距離を計算
        num_agents = Params["num_agents"]
        alpha_i = [
            self.various.Angular_distance(
                sorted_theta[idx][1],
                sorted_theta[(idx + 1) % num_agents][1],
                sorted_theta[(idx - 1) % num_agents][1],
            )[0]
            for idx in range(num_agents)
        ]

        return alpha_i

    def _save_error_data_if_enabled(self, frame_index):
        """CSV保存が有効な場合に誤差データを保存

        Args:
            frame_index: 現在のフレーム番号
        """
        if Params["save_csv"]:
            target_error, agent_errors = self.sim.get_position_errors()
            current_time = frame_index * Params["frame_time"]
            from animation import Japan_time

            # 論理順（theta順）でagent_errorsを並び替える
            # agent_index_mapping[論理インデックス] = 実際のインデックス
            agent_errors_logical = [
                agent_errors[self.ani.agent_index_mapping[j]] 
                for j in range(Params["num_agents"])
            ]
            
            save_error_csv_data(Japan_time, current_time, target_error, agent_errors_logical)

    def _process_target_movement(self, target_position):
        """ターゲットの移動処理

        Args:
            target_position: 現在のターゲット位置

        Returns:
            更新後のターゲット位置
        """
        if Params["target_move"]:
            self._update_target_position()
            # 更新したターゲット位置をCoppeliaSimに反映
            self.sim.settargetposition(self.target_position)
            # 更新後のターゲット位置を使用
            return self.target_position
        return target_position

    def _update_target_position(self):
        """ターゲットを目標地点に向けて移動させる"""
        current_pos_2d = np.array([self.target_position[0], self.target_position[1]])
        distance_to_goal = np.linalg.norm(current_pos_2d - self.target_goal)

        # 目標位置に到達済みの場合は位置を固定
        if distance_to_goal <= self.target_tolerance:
            self._set_target_at_goal()
            return

        # 目標位置に向かって移動
        direction = (self.target_goal - current_pos_2d) / distance_to_goal
        movement = direction * Params["target_move_speed"] * Params["frame_time"]

        # 移動量が残り距離より大きい場合は目標位置に直接到達
        if np.linalg.norm(movement) > distance_to_goal:
            self._set_target_at_goal()
        else:
            # 通常移動
            new_pos_2d = current_pos_2d + movement
            self._set_target_position(new_pos_2d)

    def _set_target_at_goal(self):
        """ターゲットを目標位置に設定"""
        self._set_target_position(self.target_goal)
        self._notify_target_reached()

    def _set_target_position(self, position_2d):
        """ターゲットの位置を設定（z座標は保持）

        Args:
            position_2d: 2D位置（x, y）
        """
        self.target_position = [
            float(position_2d[0]),
            float(position_2d[1]),
            float(self.target_initial_z),
        ]

    def _notify_target_reached(self):
        """ターゲット到達メッセージを表示（未到達の場合のみ）"""
        if not self.target_reached_goal:
            print(
                f"Target が目標位置 [{self.target_goal[0]}, {self.target_goal[1]}] に到達しました"
            )
            self.target_reached_goal = True

    def _compute_relative_distances(self, target_position, agent_positions):
        """全エージェントの相対距離を計算

        Args:
            target_position: ターゲット位置
            agent_positions: エージェントの位置リスト

        Returns:
            (最小相対距離, 最大相対距離)
        """
        distances = [
            self.ani.various.Distance(
                np.array(agent_positions[j]) - np.array(target_position)
            )
            for j in range(Params["num_agents"])
        ]
        return min(distances), max(distances)

    def _execute_formation_control(
        self, frame_index, target_position, agent_positions, min_ro_i, max_ro_i, alpha_i
    ):
        """フォーメーション制御を実行

        Args:
            frame_index: 現在のフレーム番号
            target_position: ターゲット位置
            agent_positions: エージェントの位置リスト
            min_ro_i: 最小相対距離
            max_ro_i: 最大相対距離
            alpha_i: 角距離リスト
        """
        PATROL_THRESHOLD = 10  # パトロール開始距離の閾値

        if min_ro_i > PATROL_THRESHOLD:
            self._handle_patrol_formation(agent_positions)
        elif min_ro_i <= PATROL_THRESHOLD and not self.circle_formation_started:
            self._handle_semicircle_formation(
                frame_index, target_position, agent_positions, max_ro_i, alpha_i
            )
        else:
            self._handle_circle_formation(frame_index, target_position, agent_positions)

    def _handle_patrol_formation(self, agent_positions):
        """パトロールフォーメーション処理

        Args:
            agent_positions: エージェントの位置リスト
        """
        # 距離が閾値より大きい場合は巡回
        self.patroll.animate(agent_positions)
        self.circle_formation_started = False  # 巡回に戻ったらリセット
        self.formation_switch_counter = 0  # カウンターもリセット
        self.transitioning_to_circle = False  # 遷移状態もリセット

    def _handle_semicircle_formation(
        self, i, target_position, agent_positions, max_ro_i, alpha_i
    ):
        """半円フォーメーション処理と円形への遷移

        Args:
            i: フレームカウンター
            target_position: ターゲット位置
            agent_positions: エージェントの位置リスト
            max_ro_i: 最大相対距離
            alpha_i: 角距離リスト
        """
        self._setup_semicircle_parameters()
        self.ani.animate(
            i, target_position, agent_positions, formation_type="semicircle"
        )

        if self.transitioning_to_circle:
            self._process_circle_transition()
        else:
            self._check_semicircle_completion(max_ro_i, alpha_i)

    def _setup_semicircle_parameters(self):
        """半円フォーメーションのパラメータを設定"""
        Params["R"] = 5
        Params["Omega"] = 0

        # 遷移中でない場合は初期値を設定
        if not self.transitioning_to_circle:
            Params["d_i"] = [
                np.pi / 5,
                np.pi / 5,
                np.pi / 5,
                np.pi / 5,
                np.pi / 5,
                5 * np.pi / 5,
            ]  # 6台の場合

    def _process_circle_transition(self):
        """円形フォーメーションへの遷移処理"""
        TARGET_D_I = np.pi / 3
        all_reached = True

        for idx in range(len(Params["d_i"])):
            current_d_i = Params["d_i"][idx]
            diff = TARGET_D_I - current_d_i

            if abs(diff) > self.transition_step_size:
                # まだ目標に到達していない
                Params["d_i"][idx] += (
                    self.transition_step_size
                    if diff > 0
                    else -self.transition_step_size
                )
                all_reached = False
            else:
                # 目標に到達
                Params["d_i"][idx] = TARGET_D_I

        # すべてのd_iが目標値に到達したら円形フォーメーションに移行
        if all_reached:
            self._complete_circle_transition()

    def _complete_circle_transition(self):
        """円形フォーメーションへの遷移を完了"""
        self.circle_formation_started = True
        self.transitioning_to_circle = False
        self.ani.mapping_initialized = False
        print("すべてのd_iがπ/3に到達しました。円形フォーメーションに移行します")

    def _check_semicircle_completion(self, max_ro_i, alpha_i):
        """半円フォーメーション完成判定

        Args:
            max_ro_i: 最大相対距離
            alpha_i: 角距離リスト
        """
        MAX_DISTANCE_THRESHOLD = 5.1
        MAX_ANGLE_THRESHOLD = 5 * np.pi / 5
        REQUIRED_CONSECUTIVE_FRAMES = 20

        # 条件が満たされた場合、カウンターを増やす
        if max_ro_i <= MAX_DISTANCE_THRESHOLD and alpha_i[5] <= MAX_ANGLE_THRESHOLD:
            self.formation_switch_counter += 1
            print(
                f"半円フォーメーション確認カウント: {self.formation_switch_counter}/{REQUIRED_CONSECUTIVE_FRAMES}"
            )
        else:
            self.formation_switch_counter = 0

        # 連続で条件が満たされたら遷移開始
        if self.formation_switch_counter >= REQUIRED_CONSECUTIVE_FRAMES:
            self.transitioning_to_circle = True
            print("半円フォーメーション完成。円形への遷移を開始します")

    def _handle_circle_formation(self, i, target_position, agent_positions):
        """円形フォーメーション処理

        Args:
            i: フレームカウンター
            target_position: ターゲット位置
            agent_positions: エージェントの位置リスト
        """
        self._setup_circle_parameters(i)
        self.ani.animate(i, target_position, agent_positions, formation_type="circle")

    def _setup_circle_parameters(self, frame_index):
        """円形フォーメーションのパラメータを設定

        Args:
            frame_index: 現在のフレーム番号
        """
        SWITCH_INTERVAL = 300  # パラメータ切り替え間隔（フレーム単位）

        Params["R"] = 4
        Params["d_i"] = [np.pi / 3] * 6  # 6台の場合

        # 300ステップごとにパラメータを切り替え
        cycle_index = (frame_index // SWITCH_INTERVAL) % len(Params["omega_values"])

        Params["Omega"] = Params["omega_values"][cycle_index]
        Params["l2"] = Params["l2_values"][cycle_index]
        Params["agent_speed_max"] = Params["agent_speed_max_values"][cycle_index]

    def _finalize_simulation(self):
        """シミュレーション終了処理"""
        print(f"\n同期モードでのシミュレーションが終了しました。")

        # シミュレーション時間を取得（停止時はNoneの可能性あり）
        final_time = self.sim.get_simulation_time()
        if final_time is not None:
            print(f"最終シミュレーション時間: {final_time:.2f}秒")

        print("CoppeliaSim を停止中...")
        self.sim.stop_simulation()
        print("シミュレーション終了")

        # CSV保存が有効な場合、プロット機能を実行
        if Params["save_csv"]:
            self._plot_saved_data()

    def _plot_saved_data(self):
        """保存されたCSVデータをプロット"""
        print("\nCSVデータをプロット中...")
        from animation import Japan_time

        plot_csv_data(Japan_time)
        print("プロット完了")


if __name__ == "__main__":
    controller = Main()
    controller.run()
