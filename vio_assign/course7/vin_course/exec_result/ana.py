#!/bin/env python

import sys
import pandas as pd
import matplotlib.pyplot as plt

def hession_cost_extract(filename):
    cost = []
    #hessian_cost_pattern = "hessian cost"
    hessian_cost_pattern = "makeHessian cost"
    with open(filename,'r') as f:
        line = f.readline()
        while line:
            if hessian_cost_pattern in line:
                #ts = line.split()[0:4]
                ts = line.split()[2]
                cost.append(float(ts))
            line = f.readline()
    return cost

def plot_data_raw_trend(data_list):
    plt.plot(data_list)
    plt.ylabel("mk hessian time cost.")
    plt.show()


def stat_of_data(data_list):
    df = pd.DataFrame(data_list)
    df.columns = ['mh_ts_cost']
    print (df.describe())

def main():
    print("len of argv", len(sys.argv))
    if len(sys.argv) <= 1:
        print( "Usage: ", __file__, " hessian_cost_file" )
        sys.exit(1)

    cost = hession_cost_extract(sys.argv[1])
    print("Total ", len(cost), "cost record.")

    stat_of_data(cost)



if __name__ == "__main__":
    main()
