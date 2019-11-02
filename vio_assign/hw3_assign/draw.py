

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    df = pd.read_csv("./iter_lamda.csv")
    

    plt.plot(df["iter"],df['val'])
    plt.show()
if __name__ == "__main__":
    main()