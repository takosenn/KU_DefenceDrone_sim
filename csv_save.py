import csv
import matplotlib.pyplot as plt
import os
from parameter import Params


def get_csv_folder_path(Japan_time):
    """CSV保存用のフォルダパスを取得（なければ作成）

    Returns:
        str: CSV/日付時間/ のパス
    """
    # CSVフォルダのパス
    csv_base_folder = "CSV"
    # 日付時間フォルダのパス
    datetime_folder = Japan_time.strftime("%Y-%m-%d-%H-%M-%S")
    full_path = os.path.join(csv_base_folder, datetime_folder)

    # フォルダが存在しない場合は作成
    if not os.path.exists(full_path):
        os.makedirs(full_path)
        print(f"CSVフォルダを作成しました: {full_path}")

    return full_path


def get_svg_folder_path():
    """SVG保存用のフォルダパスを取得（なければ作成）

    Returns:
        str: SVG/ のパス
    """
    svg_folder = "SVG"

    # フォルダが存在しない場合は作成
    if not os.path.exists(svg_folder):
        os.makedirs(svg_folder)
        print(f"SVGフォルダを作成しました: {svg_folder}")

    return svg_folder


def set_csv_header(Japan_time, nam):
    csv_folder = get_csv_folder_path(Japan_time)
    csv_path = os.path.join(csv_folder, f"name[{nam}].csv")

    with open(
        csv_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as nm:
        writer = csv.writer(nm)
        writer.writerow(
            [
                "Time",
                "Agent[1]",
                "Agent[2]",
                "Agent[3]",
                "Agent[4]",
                "Agent[5]",
                "Agent[6]",
            ]
        )


def save_csv_data(Japan_time, current_time, name, nam):
    csv_folder = get_csv_folder_path(Japan_time)
    csv_path = os.path.join(csv_folder, f"name[{nam}].csv")

    with open(
        csv_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as nm:
        writer = csv.writer(nm)
        writer.writerow(
            [
                current_time,
                name[0],
                name[1],
                name[2],
                name[3],
                name[4],
                name[5],
            ]
        )  # データをCSVに書き込む


def set_error_csv_header(Japan_time):
    """誤差データ用のCSVヘッダーを設定"""
    csv_folder = get_csv_folder_path(Japan_time)

    # Target誤差用
    target_error_path = os.path.join(csv_folder, "name[target_error].csv")
    with open(
        target_error_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Error_X", "Error_Y", "Error_Z", "Error_Norm"])

    # Agent誤差用（全Agentを1つのファイルに）
    # 誤差ノルムのみ
    agents_error_norm_path = os.path.join(csv_folder, "name[agents_error_norm].csv")
    with open(
        agents_error_norm_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        header = ["Time"] + [f"Agent[{i}]" for i in range(Params["num_agents"])]
        writer.writerow(header)

    # XYZ成分も含めた詳細版
    agents_error_detail_path = os.path.join(csv_folder, "name[agents_error_detail].csv")
    with open(
        agents_error_detail_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        header = ["Time"]
        for i in range(Params["num_agents"]):
            header.extend(
                [f"Agent[{i}]_X", f"Agent[{i}]_Y", f"Agent[{i}]_Z", f"Agent[{i}]_Norm"]
            )
        writer.writerow(header)


def save_error_csv_data(Japan_time, current_time, target_error, agent_errors):
    """誤差データをCSVに保存

    Parameters:
    -----------
    Japan_time : datetime
        CSVファイル名に使用する日時
    current_time : float
        現在のシミュレーション時間
    target_error : np.ndarray
        Targetの誤差 [x, y, z]
    agent_errors : list of np.ndarray
        各Agentの誤差リスト [[x, y, z], ...]
    """
    import numpy as np

    csv_folder = get_csv_folder_path(Japan_time)

    # Target誤差を保存
    target_error_norm = np.linalg.norm(target_error)
    target_error_path = os.path.join(csv_folder, "name[target_error].csv")
    with open(
        target_error_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                current_time,
                target_error[0],
                target_error[1],
                target_error[2],
                target_error_norm,
            ]
        )

    # 全Agent誤差を1つのファイルに保存 (誤差ノルムのみ)
    agents_error_norm_path = os.path.join(csv_folder, "name[agents_error_norm].csv")
    with open(
        agents_error_norm_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        error_norms = [current_time] + [
            np.linalg.norm(agent_error) for agent_error in agent_errors
        ]
        writer.writerow(error_norms)

    # 全Agent誤差を1つのファイルに保存 (XYZ成分も含めた詳細版)
    agents_error_detail_path = os.path.join(csv_folder, "name[agents_error_detail].csv")
    with open(
        agents_error_detail_path,
        mode="a",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        row = [current_time]
        for agent_error in agent_errors:
            row.extend(
                [
                    agent_error[0],
                    agent_error[1],
                    agent_error[2],
                    np.linalg.norm(agent_error),
                ]
            )
        writer.writerow(row)


def plot_csv_data(Japan_time, data_names=None):
    """
    実行終了後にCSVファイルを読み込んでプロットする

    Parameters:
    -----------
    Japan_time : datetime
        CSVファイル名に使用する日時
    data_names : list of str, optional
        プロットするデータ名のリスト（例: ["ro_i", "theta", "omega_i"]）
        Noneの場合は全てのCSVファイルをプロット
    """
    if data_names is None:
        # デフォルトでプロットするデータ
        data_names = [
            "ro_i",
            "theta",
            "alpha_i",
            "omega_i",
            "eta",
            "e_i_1",
            "e_i_2",
            "fi",
        ]

    # プロット用の設定
    num_plots = len(data_names)
    cols = 2  # 2列で表示
    rows = (num_plots + 1) // 2  # 必要な行数

    fig, axes = plt.subplots(rows, cols, figsize=(15, 5 * rows))
    if num_plots == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    csv_folder = get_csv_folder_path(Japan_time)

    for idx, nam in enumerate(data_names):
        csv_filename = os.path.join(csv_folder, f"name[{nam}].csv")

        # CSVファイルが存在するか確認
        if not os.path.exists(csv_filename):
            print(f"警告: {csv_filename} が見つかりません。スキップします。")
            continue

        # CSVファイルを読み込む
        time_data = []
        agent_data = [[] for _ in range(Params["num_agents"])]

        with open(csv_filename, mode="r", encoding="utf-8") as f:
            reader = csv.reader(f)
            next(reader)  # ヘッダーをスキップ

            for row in reader:
                time_data.append(float(row[0]))
                for i in range(Params["num_agents"]):
                    agent_data[i].append(float(row[i + 1]))

        # プロット
        ax = axes[idx]
        for i in range(Params["num_agents"]):
            ax.plot(time_data, agent_data[i], label=f"Agent[{i}]")

        ax.set_xlabel("Time [s]")
        ax.set_ylabel(nam)
        ax.set_title(f"{nam} vs Time")
        ax.legend()
        ax.grid(True)

    # 使用しないサブプロットを非表示
    for idx in range(num_plots, len(axes)):
        fig.delaxes(axes[idx])

    plt.tight_layout()

    # SVGフォルダに画像として保存
    svg_folder = get_svg_folder_path()
    plot_filename = os.path.join(
        svg_folder, f"plot_{Japan_time.strftime('%Y-%m-%d-%H-%M-%S')}.svg"
    )
    plt.savefig(plot_filename)
    print(f"\nプロットを保存しました: {plot_filename}")

    # メモリを解放
    plt.close(fig)
