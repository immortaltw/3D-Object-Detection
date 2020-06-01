import pandas as pd
import numpy as np
import copy
import os
import sys
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns

def parse():
    path = "report/"
    filelist = os.listdir("report/")
    lidar_ttc = np.array([12.2879,12.5156,17.8431,15.4642,12.4275,12.8097,12.9415,13.2442,13.3569,12.3251,11.914,10.513,8.91807,9.75289,8.08422,9.25433,11.4098,8.03695])
    result = []
    res_df = pd.DataFrame()
    for fp in filelist:
        if fp.endswith(".csv") and os.path.getsize(path + fp) > 0:
            stats = pd.read_csv(path + fp, delimiter=',', header=None).replace(-np.inf, np.nan)
            stats['combination'] = os.path.splitext(fp)[0]
            result.append(stats)

    res_df = pd.concat(result, ignore_index=True)
    res_df.drop(res_df.columns[18], inplace=True, axis=1)

    res_df = res_df.transpose()
    res_df.columns = res_df.iloc[-1]
    res_df = res_df[:-1]

    # Drop any detector/descriptor combinations that contains NaN values
    res_df=res_df.dropna(axis=1,how='any')
    res_df.to_csv("final_report.csv", index=None)

    # Plot everyone
    res_df.plot()

    # Sort by std
    sorted_df = res_df.std(axis=0).sort_values()

    print(sorted_df)

    plt.figure(figsize=(20,9))
    sns.set_style("darkgrid")
    x = np.linspace(1, 18, 18)

    # Plot best 5 combos
    for comb in sorted_df.index[0:5]:
        plt.plot(x, res_df[comb], 
                linewidth=1.0
                )

    # Plot lidar
    plt.plot(x, lidar_ttc, 
            linewidth=1.0
            )

    plt.legend(np.hstack([sorted_df.index.values[:5], np.array(["Lidar"])]))
    plt.title("Best 5 detector/descriptor combo and Lidar")
    plt.xlabel("Image ID")
    plt.ylabel("TTC (sec)")
    plt.show()

if __name__ == "__main__":
    parse()