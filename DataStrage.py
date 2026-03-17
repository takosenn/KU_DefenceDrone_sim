# 時間積分値を保持するためのリスト

from parameter import Params

# e_i_1, e_i_2の時間積分値（各エージェントごと）
e_i_1_integral = [0.0 for _ in range(int(Params["num_agents"]))]
e_i_2_integral = [0.0 for _ in range(int(Params["num_agents"]))]
