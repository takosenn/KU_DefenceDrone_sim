import numpy as np
from parameter import Params
from connect_Coppelia import Simulation


class Patroll:
    def __init__(self):
        self.sim = Simulation()
        self.agent_positions = []
        # 巡回開始位置（x, y座標）
        self.patrol_start_positions = np.array(
            [[0.0, 0.0], [5.0, 0.0], [7.5, 2.5], [7.5, 7.5], [2.5, 7.5], [0.0, 5.0]]
        )
        self.patrol_speed = 1  # 移動速度[m/s]
        self.reached_start_position = [False] * Params[
            "num_agents"
        ]  # 各エージェントが巡回開始位置に到達したか
        self.tolerance = 0.1  # 到達判定の許容誤差[m]

        # 巡回動作用の変数
        self.current_target_index = list(
            range(Params["num_agents"])
        )  # 各エージェントが目指している位置のインデックス
        self.patrol_started = False  # 巡回動作が開始されたか

        # 正方形の角の座標
        self.square_corners = {
            "bottom_left": np.array([0.0, 0.0]),
            "bottom_right": np.array([7.5, 0.0]),
            "top_right": np.array([7.5, 7.5]),
            "top_left": np.array([0.0, 7.5]),
        }

        # 各エージェントの巡回経路（反時計回りに正方形の角を巡回）
        self.patrol_routes = self._generate_patrol_routes()

        # 各エージェントの中間ウェイポイント（角を経由する必要がある場合）
        self.waypoints = [None] * Params["num_agents"]

    def _generate_patrol_routes(self):
        """各エージェントの巡回経路を生成（初期位置から反時計回りに正方形の角を巡回）"""
        # 正方形の角を反時計回りに並べたリスト
        corners_ccw = [
            np.array([0.0, 0.0]),  # bottom_left
            np.array([7.5, 0.0]),  # bottom_right
            np.array([7.5, 7.5]),  # top_right
            np.array([0.0, 7.5]),  # top_left
        ]

        patrol_routes = []

        for j in range(Params["num_agents"]):
            start_pos = self.patrol_start_positions[j]
            route = []

            # 初期位置から最も近い角を見つける
            min_distance = float("inf")
            nearest_corner_idx = 0

            for i, corner in enumerate(corners_ccw):
                distance = np.linalg.norm(start_pos - corner)
                if distance < min_distance:
                    min_distance = distance
                    nearest_corner_idx = i

            # 初期位置が角にない場合は、その次の角から巡回を開始
            # 初期位置が角にある場合は、その次の角から開始
            if min_distance < 0.1:  # 角にいる場合
                start_idx = (nearest_corner_idx + 1) % 4
            else:  # 角にいない場合は、次の角（反時計回り）から開始
                # 初期位置がどの辺上にあるかを判定し、次の角を決定
                start_idx = self._get_next_corner_index(start_pos, corners_ccw)

            # 反時計回りに角を巡回し、最後に初期位置に戻る
            for i in range(4):
                corner_idx = (start_idx + i) % 4
                route.append(corners_ccw[corner_idx].copy())

            # 最後に初期位置に戻る
            route.append(start_pos.copy())

            patrol_routes.append(route)

        return patrol_routes

    def _get_next_corner_index(self, pos, corners_ccw):
        """位置から反時計回りの次の角のインデックスを返す"""
        x, y = pos

        # 各辺上の位置から次の角へのマッピング（反時計回り）
        if abs(y - 0.0) < 0.5 and 0.0 <= x < 7.5:  # 下辺
            return 1  # bottom_right
        elif abs(x - 7.5) < 0.5 and 0.0 <= y < 7.5:  # 右辺
            return 2  # top_right
        elif abs(y - 7.5) < 0.5 and 0.0 < x <= 7.5:  # 上辺
            return 3  # top_left
        elif abs(x - 0.0) < 0.5 and 0.0 < y <= 7.5:  # 左辺
            return 0  # bottom_left
        else:
            # デフォルト：最も近い角の次
            min_distance = float("inf")
            nearest_idx = 0
            for i, corner in enumerate(corners_ccw):
                distance = np.linalg.norm(pos - corner)
                if distance < min_distance:
                    min_distance = distance
                    nearest_idx = i
            return (nearest_idx + 1) % 4

    def animate(self, agent_positions):
        self.agent_positions = agent_positions

        # 全エージェントが巡回開始位置に到達したかチェック
        if all(self.reached_start_position):
            if not self.patrol_started:
                print(
                    "全エージェントが巡回開始位置に到達しました。巡回動作を開始します。"
                )
                self.patrol_started = True
                # 各エージェントの次の目標位置を最初のルートポイントに設定
                for j in range(Params["num_agents"]):
                    self.current_target_index[j] = 0

            # 巡回動作を実行
            self._patrol_movement(agent_positions)
            return

        # 巡回開始位置への移動
        new_positions = []
        for j in range(Params["num_agents"]):
            current_pos = np.array(agent_positions[j])  # [x, y, z]
            target_pos_2d = self.patrol_start_positions[j]  # [x, y]

            # z座標は現在の値を維持
            target_pos_3d = np.array(
                [target_pos_2d[0], target_pos_2d[1], current_pos[2]]
            )

            # 到達判定（xy平面のみ）
            distance_2d = np.linalg.norm(current_pos[:2] - target_pos_2d)

            if distance_2d < self.tolerance:
                # 到達済み
                if not self.reached_start_position[j]:
                    self.reached_start_position[j] = True
                    print(f"Agent[{j}] が巡回開始位置に到達しました")
                new_positions.append(target_pos_3d.tolist())
            else:
                # 移動中
                direction = target_pos_3d - current_pos
                direction_2d_norm = np.linalg.norm(direction[:2])

                if direction_2d_norm > 0:
                    # 正規化した方向ベクトル
                    direction_normalized = direction / direction_2d_norm
                    # 移動量（速度 * 時間）
                    movement = (
                        direction_normalized * self.patrol_speed * Params["frame_time"]
                    )

                    # 移動量が残り距離より大きい場合は、目標位置に直接移動
                    if np.linalg.norm(movement) > distance_2d:
                        new_positions.append(target_pos_3d.tolist())
                    else:
                        new_positions.append((current_pos + movement).tolist())
                else:
                    new_positions.append(current_pos.tolist())

        # CoppeliaSim に位置を反映
        self.sim.setAgentposition(new_positions)

    def _patrol_movement(self, agent_positions):
        """反時計回りに正方形の角を巡回する動作"""
        new_positions = []

        for j in range(Params["num_agents"]):
            current_pos = np.array(agent_positions[j])  # [x, y, z]
            target_idx = self.current_target_index[j]

            # 各エージェントの巡回ルートから目標位置を取得
            route = self.patrol_routes[j]
            target_pos_2d = route[target_idx]  # [x, y]

            # z座標は現在の値を維持
            target_pos_3d = np.array(
                [target_pos_2d[0], target_pos_2d[1], current_pos[2]]
            )

            # 目標位置までの距離（xy平面のみ）
            distance_2d = np.linalg.norm(current_pos[:2] - target_pos_2d)

            if distance_2d < self.tolerance:
                # 目標位置に到達したので、次の目標位置に更新
                next_idx = (target_idx + 1) % len(route)
                self.current_target_index[j] = next_idx

                # ループ完了の通知（初期位置に戻った場合）
                if next_idx == 0:
                    print(f"Agent[{j}] が一周して初期位置に戻りました")

                new_positions.append(target_pos_3d.tolist())
            else:
                # 目標位置に向かって移動
                new_positions.append(
                    self._move_towards(current_pos, target_pos_3d, distance_2d)
                )

        # CoppeliaSim に位置を反映
        self.sim.setAgentposition(new_positions)

    def _move_towards(self, current_pos, target_pos_3d, distance_2d):
        """目標位置に向かって移動"""
        direction = target_pos_3d - current_pos
        direction_2d_norm = np.linalg.norm(direction[:2])

        if direction_2d_norm > 0:
            # 正規化した方向ベクトル
            direction_normalized = direction / direction_2d_norm
            # 移動量（速度 * 時間）
            movement = direction_normalized * self.patrol_speed * Params["frame_time"]

            # 移動量が残り距離より大きい場合は、目標位置に直接移動
            if np.linalg.norm(movement) > distance_2d:
                return target_pos_3d.tolist()
            else:
                return (current_pos + movement).tolist()
        else:
            return current_pos.tolist()
