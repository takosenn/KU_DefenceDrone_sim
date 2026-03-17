"""
複数のパラメータ設定を自動的に実行するスクリプト

使用方法:
    python run_experiments.py
    または
    python run_experiments.py --config custom_config.yaml
"""

import yaml
import sys
import time
import os
from datetime import datetime
from parameter import Params, update_params
import numpy as np


def expand_parameter_sweep(experiment):
    """パラメータスイープを展開して複数の実験を生成"""
    if "parameter_sweep" not in experiment:
        return [experiment]

    sweep_config = experiment["parameter_sweep"]
    name_template = experiment.get("name_template", "exp_{value}")
    fixed_params = experiment.get("fixed_parameters", {})

    # 各パラメータの値リストを生成
    param_value_lists = {}

    for param_name, sweep_range in sweep_config.items():
        if isinstance(sweep_range, dict):
            # 範囲指定 (start, end, step)
            start = sweep_range.get("start")
            end = sweep_range.get("end")
            step = sweep_range.get("step", 1)

            # 値のリストを生成
            current = start
            values = []
            while current <= end + 1e-9:  # 浮動小数点誤差対策
                values.append(round(current, 10))  # 丸め誤差を防ぐ
                current += step
        elif isinstance(sweep_range, list):
            # リスト指定
            values = sweep_range
        else:
            values = [sweep_range]

        param_value_lists[param_name] = values

    # 複数パラメータの直積（全組み合わせ）を生成
    import itertools

    param_names = list(param_value_lists.keys())
    param_values = list(param_value_lists.values())

    experiments = []
    for value_combination in itertools.product(*param_values):
        # 実験名を生成
        exp_name = name_template
        parameters = fixed_params.copy()

        for param_name, value in zip(param_names, value_combination):
            # omega_values がリストの場合はそのまま使用
            if param_name == "omega_values":
                parameters[param_name] = value
                # 名前表示用にリストを文字列化（例: [0.1, 0.1] -> "0.1_0.1"）
                if isinstance(value, list):
                    value_str = "_".join(str(v) for v in value)
                else:
                    value_str = str(value)
                exp_name = exp_name.replace(f"{{{param_name}}}", value_str)
            else:
                parameters[param_name] = value
                exp_name = exp_name.replace(f"{{{param_name}}}", str(value))

        experiments.append({"name": exp_name, "parameters": parameters})

    return experiments


def load_config(config_file="config.yaml"):
    """YAMLファイルから設定を読み込む"""
    try:
        with open(config_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        # パラメータスイープを展開
        if "experiments" in config:
            expanded_experiments = []
            for exp in config["experiments"]:
                expanded_experiments.extend(expand_parameter_sweep(exp))
            config["experiments"] = expanded_experiments

        return config
    except FileNotFoundError:
        print(f"エラー: {config_file} が見つかりません")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"エラー: YAML読み込みに失敗しました: {e}")
        sys.exit(1)


def run_single_experiment(experiment_config):
    """単一の実験を実行する"""
    experiment_name = experiment_config["name"]
    parameters = experiment_config["parameters"]

    print("\n" + "=" * 70)
    print(f"実験開始: {experiment_name}")
    print(f"開始時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # パラメータを表示
    print("\n実験パラメータ:")
    for key, value in parameters.items():
        print(f"  {key}: {value}")
    print()

    # パラメータを更新
    update_params(parameters)

    # d_i を num_agents に基づいて自動設定
    if Params["num_agents"] > 0:
        d_i_value = 2 * np.pi / Params["num_agents"]
        Params["d_i"] = [d_i_value] * Params["num_agents"]
        print(f"d_i を自動設定: {Params['d_i']}")

    # main.py を実行
    try:
        # main.py のモジュールを再読み込みして、新しいパラメータで実験を実行
        import importlib
        import sys

        # main モジュールが既に読み込まれている場合は削除
        if "main" in sys.modules:
            del sys.modules["main"]

        # animation も再読み込みが必要
        if "animation" in sys.modules:
            del sys.modules["animation"]

        # main.py をインポート
        import main

        # Main クラスのインスタンスを作成してシミュレーションを実行
        controller = main.Main()
        controller.run()

        print(f"\n実験 '{experiment_name}' が正常に完了しました")

    except KeyboardInterrupt:
        print(f"\n実験 '{experiment_name}' がユーザーによって中断されました")
        raise
    except Exception as e:
        print(f"\nエラー: 実験 '{experiment_name}' の実行中に問題が発生しました")
        print(f"エラー内容: {type(e).__name__}: {e}")
        return False

    print("=" * 70)
    print(f"実験終了: {experiment_name}")
    print(f"終了時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70 + "\n")

    return True


def main():
    """メイン関数: 複数の実験を順次実行"""
    # コマンドライン引数から設定ファイルを取得
    config_file = "config.yaml"
    if len(sys.argv) > 2 and sys.argv[1] == "--config":
        config_file = sys.argv[2]

    print("=" * 70)
    print("複数パラメータ実験システム")
    print("=" * 70)
    print(f"設定ファイル: {config_file}")
    print(f"実行開始時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    # 設定を読み込み
    config = load_config(config_file)

    experiments = config.get("experiments", [])
    execution_config = config.get("execution", {})

    if not experiments:
        print("エラー: 実行する実験が定義されていません")
        sys.exit(1)

    print(f"実行予定の実験数: {len(experiments)}\n")

    # 各実験のリストを表示
    print("実行予定の実験:")
    for i, exp in enumerate(experiments, 1):
        print(f"  {i}. {exp['name']}")
    print()

    # 実行確認
    if len(experiments) > 1:
        response = input("すべての実験を実行しますか？ (y/n): ")
        if response.lower() != "y":
            print("実験を中止しました")
            sys.exit(0)

    # 各実験を順次実行
    successful_count = 0
    failed_count = 0
    wait_time = execution_config.get("wait_between_experiments", 5)

    for i, experiment in enumerate(experiments, 1):
        print(f"\n進行状況: {i}/{len(experiments)}")

        try:
            success = run_single_experiment(experiment)
            if success:
                successful_count += 1
            else:
                failed_count += 1

            # 最後の実験でなければ待機
            if i < len(experiments):
                print(f"\n次の実験まで {wait_time} 秒待機中...")
                time.sleep(wait_time)

        except KeyboardInterrupt:
            print("\n\nすべての実験が中断されました")
            break

    # 最終結果を表示
    print("\n" + "=" * 70)
    print("すべての実験が完了しました")
    print("=" * 70)
    print(f"成功: {successful_count} 件")
    print(f"失敗: {failed_count} 件")
    print(f"合計: {len(experiments)} 件")
    print(f"終了時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)


if __name__ == "__main__":
    main()
