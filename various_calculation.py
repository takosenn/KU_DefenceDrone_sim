# ローカル座標系の値をグローバル座標系に変換する関数

import numpy as np
from parameter import Params
from DataStrage import e_i_1_integral, e_i_2_integral


class Various:
    def __init__(self):
        self.theta = []
        for i in range(Params["num_agents"]):
            theta = i * np.pi / 3
            self.theta.append(theta)
        self.alpha = []
        for i in range(Params["num_agents"]):
            alpha = i * np.pi / 3
            self.alpha.append(alpha)
        self.alpha_minus = np.copy(self.alpha)

    def Theta(self, pos):
        self.theta = np.arctan2(pos[1], pos[0])
        if self.theta >= 0:
            self.theta = self.theta
        else:
            self.theta = self.theta + 2 * np.pi
        return self.theta

    def Angular_distance(
        self, theta, theta_plus, theta_minus
    ):  # theta_minusは(i-1)番目、theta_plusは(i+1)番目の角度(World座標系)
        diff_plus = theta_plus - theta
        if diff_plus >= 0:
            alpha = diff_plus
        else:
            alpha = diff_plus + (2 * np.pi)
        diff_minus = theta - theta_minus
        if diff_minus >= 0:
            alpha_minus = diff_minus
        else:
            alpha_minus = diff_minus + (2 * np.pi)
        return alpha, alpha_minus

    def Angular_velocity(
        self, theta, prev_theta
    ):  # prev_thetaはi番目の角度 theta = 2πの際に問題あり
        delta = theta - prev_theta
        if delta > np.pi:
            delta -= 2 * np.pi
        elif delta < -np.pi:
            delta += 2 * np.pi
        omega = delta / Params["frame_time"]
        return omega

    def Velocity(self, current_pos, prev_pos):
        velocity = (np.array(current_pos) - np.array(prev_pos)) / Params["frame_time"]
        return velocity

    def Distance(self, local_pos):
        """ローカル座標系での位置ベクトルから相対距離を計算"""
        distance = np.linalg.norm(local_pos)
        return distance

    def coordinate_trans(self, theta_global, u):
        A = np.array(
            [
                [np.cos(theta_global), -np.sin(theta_global)],
                [np.sin(theta_global), np.cos(theta_global)],
            ]
        )
        u_vec_local = np.array([u[0], u[1]])
        u_vec = A @ u_vec_local
        return u_vec

    def caluculate(
        self,
        i,
        j,
        alpha_i,
        alpha_i_minus,
        omega_i_plus,
        omega_i,
        omega_i_minus,
        ro_i,
        eta,
        logical_j=None,
    ):
        """u_r(放射方向)とu_theta(接線方向)を計算する

        役割：
        - 各エージェントの制御入力（放射方向・接線方向の速度成分）を計算
        - フォーメーション維持のための誤差項を計算・積分
        - 論理インデックスに基づく理想相対角距離を使用

        Args:
            i: フレーム番号
            j: エージェントの実際のインデックス
            alpha_i: 隣接エージェント間の角距離
            alpha_i_minus: 前方隣接エージェントとの角距離
            omega_i_plus: 次のエージェントの角速度
            omega_i: 現在のエージェントの角速度
            omega_i_minus: 前のエージェントの角速度
            ro_i: ターゲットからの相対距離
            eta: 距離の時間微分
            logical_j: 論理インデックス（theta順の位置）

        Returns:
            (u_r, u_theta, e_i_1_積分値, e_i_2_積分値, fi)
        """

        # --- fi, zi の計算と表示 ---
        # logical_jが指定されている場合は論理インデックス（theta順）でd_iを参照
        if logical_j is not None:
            d_i_j = Params["d_i"][logical_j]  # 論理エージェントjの理想相対角距離
            logical_j_minus = (logical_j - 1) % Params["num_agents"]
            d_i_j_minus = Params["d_i"][
                logical_j_minus
            ]  # 論理エージェント(j-1)の理想相対角距離
        else:
            # 後方互換性のため、logical_jが指定されていない場合は実際のインデックスを使用
            d_i_j = Params["d_i"][j]
            d_i_j_minus = Params["d_i"][(j - 1) % Params["num_agents"]]

        fi = (d_i_j_minus * alpha_i - d_i_j * alpha_i_minus) / (d_i_j + d_i_j_minus)
        zi = (
            d_i_j_minus * (omega_i_plus - omega_i) - d_i_j * (omega_i - omega_i_minus)
        ) / (d_i_j + d_i_j_minus)

        # e_i_1, e_i_2の初期値は0、それ以降は式で計算

        if i == 0 or i == 1 or i == 2 or i == 3 or i == 4 or i == 5:
            tau_i_1 = 0
            tau_i_2 = 0
        else:
            tau_i_1 = 2
            tau_i_2 = 2
        e_i_1 = tau_i_1 * np.linalg.norm(ro_i - Params["R"] + eta)
        e_i_2 = tau_i_2 * np.linalg.norm(ro_i * (omega_i - Params["Omega"] - fi))

        # --- e_i_1, e_i_2の時間積分 ---
        e_i_1_integral[j] += e_i_1 * Params["frame_time"]
        e_i_2_integral[j] += e_i_2 * Params["frame_time"]

        # print(f"e_i_1: {e_i_1_integral[j]}")
        # print(f"e_i_2: {e_i_2_integral[j]}")
        # 論文の式(21)に従った制御プロトコルの計算

        # --- 制御プロトコルu_iの計算（時間積分したe_i_1, e_i_2を使用） ---
        # u_rが放射方向(targetに近づく離れる)の速度成分、u_thetaが接線方向の速度成分
        u_r = (
            -ro_i * omega_i**2 - eta - Params["l1"] * np.sign(ro_i - Params["R"] + eta)
        )
        u_theta = (
            (omega_i + Params["Omega"] + fi) * eta
            + zi * ro_i
            + Params["l2"] * np.sign(fi + Params["Omega"] - omega_i)
        )

        return u_r, u_theta, e_i_1_integral[j], e_i_2_integral[j], fi
