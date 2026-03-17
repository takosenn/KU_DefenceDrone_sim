from parameter import Params
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # type: ignore[import]
import time


class Simulation:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.Drone_handles = []
        self.Agent_handles = []
        self.target_Drone_handle = self.sim.getObject("/Quadcopter[0]")
        self.target_handle = self.sim.getObject("/Quadcopter[0]/target")

    def start_simulation(self):
        """シミュレーションを開始する"""
        self.sim.startSimulation()
        print("Simulationを開始")

    def stop_simulation(self):
        """シミュレーションを停止する"""
        self.sim.stopSimulation()
        print("Simulationを停止")

    def step_simulation(self):
        """シミュレーションを1ステップ進める（同期モード用）"""
        self.sim.step()

    def get_simulation_time(self):
        return self.sim.getSimulationTime()

    def get_handles(self, num_agents):
        print("CoppeliaSim からハンドルを取得中...")
        self.target_handle = self.sim.getObject("/Quadcopter[0]/target")
        self.target_Drone_handle = self.sim.getObject("/Quadcopter[0]")
        print("取得: Quadcopter[0] のtarget")
        # 各Agentの緑の球(target)のハンドル
        for i in range(num_agents):
            object_name = f"Quadcopter[{i+1}]"
            Drone_handle = self.sim.getObject(f"/{object_name}")
            self.Drone_handles.append(Drone_handle)
            Agent_handle = self.sim.getObject(f"/{object_name}/target")
            self.Agent_handles.append(Agent_handle)
            print(f"取得: {object_name} のtarget")

    def setAgentposition(self, Agents_pos_3d):
        # Coppeliasim側でAgentの緑の球(target)の位置同期
        for j in range(Params["num_agents"]):
            self.sim.setObjectPosition(self.Agent_handles[j], -1, Agents_pos_3d[j])

    def settargetposition(self,target_pos_3d):
        # Coppeliasim側でtargetの緑の球(target)の位置同期
        self.sim.setObjectPosition(self.target_handle, -1, target_pos_3d)

    def get_Drone_position(self):
        """座標を取得（CoppeliaSim または モーションキャプチャ）"""
        if Params["use_mocap"]:
            return self._get_position_from_mocap()
        else:
            return self._get_position_from_coppelia()

    def _get_position_from_coppelia(self):
        """CoppeliaSim から座標を取得"""
        Agent_positions = np.zeros((Params["num_agents"], 3))
        target_position = self.sim.getObjectPosition(self.target_handle, -1)
        for agent_idx in range(Params["num_agents"]):
            Agent_positions[agent_idx] = self.sim.getObjectPosition(
                self.Drone_handles[agent_idx], -1
            )
        return target_position, Agent_positions

    def _get_position_from_mocap(self):
        """モーションキャプチャから座標を取得

        Drone[0] -> target
        Drone[1]~[6] -> Agent[0]~[5]

        TODO: モーションキャプチャシステムとの接続を実装
        """
        # TODO: 実際のモーションキャプチャシステムから座標を取得する処理を実装
        # 例: mocap_data = self.mocap_system.get_rigid_bodies()

        Agent_positions = np.zeros((Params["num_agents"], 3))
        target_position = np.zeros(3)

        # TODO: 以下を実際のモーションキャプチャデータで置き換える
        # target_position = mocap_data['Drone[0]']  # Drone[0] -> target
        # for agent_idx in range(Params["num_agents"]):
        #     Agent_positions[agent_idx] = mocap_data[f'Drone[{agent_idx+1}]']  # Drone[1]~[6] -> Agent[0]~[5]

        raise NotImplementedError("モーションキャプチャからの座標取得は未実装です")

        return target_position, Agent_positions

    def get_position_errors(self):
        """Quadcopterの実際の位置とターゲットボールの位置の誤差を計算

        Returns:
            target_error: Target Quadcopter の誤差 [x, y, z]
            agent_errors: 各 Agent の誤差リスト [[x, y, z], ...]
        """
        # Target の誤差を計算
        target_actual_pos = self.sim.getObjectPosition(self.target_Drone_handle, -1)
        target_ball_pos = self.sim.getObjectPosition(self.target_handle, -1)
        target_error = np.array(target_actual_pos) - np.array(target_ball_pos)

        # 各 Agent の誤差を計算
        agent_errors = []
        for agent_idx in range(Params["num_agents"]):
            agent_actual_pos = self.sim.getObjectPosition(
                self.Drone_handles[agent_idx], -1
            )
            agent_ball_pos = self.sim.getObjectPosition(
                self.Agent_handles[agent_idx], -1
            )
            agent_error = np.array(agent_actual_pos) - np.array(agent_ball_pos)
            agent_errors.append(agent_error)

        return target_error, agent_errors

    def get_Drone_Velocity(self):
        """速度を取得（CoppeliaSim または モーションキャプチャ）"""
        if Params["use_mocap"]:
            return self._get_velocity_from_mocap()
        else:
            return self._get_velocity_from_coppelia()

    def _get_velocity_from_coppelia(self):
        """CoppeliaSim から速度を取得"""
        Agent_velocities = np.zeros((Params["num_agents"], 3))
        # getObjectVelocity は (線形速度, 角速度) のタプルを返すため、[0]で線形速度のみ取得
        target_velocity = self.sim.getObjectVelocity(self.target_handle)[0]
        for agent_idx in range(Params["num_agents"]):
            Agent_velocities[agent_idx] = self.sim.getObjectVelocity(
                self.Drone_handles[agent_idx]
            )[0]
        return target_velocity, Agent_velocities

    def _get_velocity_from_mocap(self):
        """モーションキャプチャから速度を取得"""
