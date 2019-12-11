#!/usr/bin/env python

import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    print("Plot the ratio.")
    file_n = sys.argv[1]
    col_n = sys.argv[2]
    print("file name: " , file_n)

    df = pd.read_csv(file_n)
    

    plt.plot(df[col_n],df['ratio'])
    plt.show()


if __name__ == "__main__":
    main()