

import matplotlib.pyplot as plt
import pandas as pd

def read_csv(f_path):
    df = pd.read_csv(f_path,header=None)
    df.columns = ['mh_time']
    df[['mh_time']] = df[['mh_time']].apply(pd.to_numeric)
    return df

def main():
    print(" This is the main")
    base_f = "no_tp.csv"
    f_1 = "1_tp_work.csv"
    f_5 = "5_tp_work.csv"
    f_10 = "10_tp_work.csv"
    f_20 = "20_tp_work.csv"
    f_base = read_csv(base_f)
    print(f_base.dtypes)
    print("base: ")
    print(f_base.describe())

    csv_1 = read_csv(f_1)
    print("single thread:")
    print (csv_1.describe())

    csv_5 = read_csv(f_5)
    print("5 thread pool:")
    print(csv_5.describe())

    csv_10 = read_csv(f_10)
    print("10 thread pool:")
    print(csv_10.describe())

    csv_20 = read_csv(f_20)
    print("20 thread pool:")
    print(csv_20.describe())

    df = pd.DataFrame()
    df['base_mh_time'] = f_base['mh_time']
    df["1_thread_mh_time"] = csv_1["mh_time"]
    df["5_thread_mh_time"] = csv_5["mh_time"]
    df["10_thread_mh_time"] = csv_10["mh_time"]
    df["20_thread_mh_time"] = csv_20["mh_time"]
    #base_tm = base_f[["mh_time"]]
    #plt.plot(f_base['mh_time'],)
    #plt.plot(csv_1['mh_time'])
    df.plot()
    plt.show()


if __name__ == "__main__":
    main()