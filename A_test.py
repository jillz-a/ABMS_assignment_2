import itertools as it

from bisect import bisect_left
from typing import List
import xlrd
import numpy as np
import pandas as pd
import scipy.stats as ss

from pandas import Categorical


def VD_A(treatment: List[float], control: List[float]):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["N", "S", "M", "L"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude


def VD_A_DF(data, val_col: str = None, group_col: str = None, sort=True):
    """
    :param data: pandas DataFrame object
        An array, any object exposing the array interface or a pandas DataFrame.
        Array must be two-dimensional. Second dimension may vary,
        i.e. groups may have different lengths.
    :param val_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains values.
    :param group_col: str, optional
        Must be specified if `a` is a pandas DataFrame object.
        Name of the column that contains group names.
    :param sort : bool, optional
        Specifies whether to sort DataFrame by group_col or not. Recommended
        unless you sort your data manually.
    :return: stats : pandas DataFrame of effect sizes
    Stats summary ::
    'A' : Name of first measurement
    'B' : Name of second measurement
    'estimate' : effect sizes
    'magnitude' : magnitude
    """

    x = data.copy()
    if sort:
        x[group_col] = Categorical(x[group_col], categories=x[group_col].unique(), ordered=True)
        x.sort_values(by=[group_col, val_col], ascending=True, inplace=True)

    groups = x[group_col].unique()

    # Pairwise combinations
    g1, g2 = np.array(list(it.combinations(np.arange(groups.size), 2))).T

    # Compute effect size for each combination
    ef = np.array([VD_A(list(x[val_col][x[group_col] == groups[i]].values),
                        list(x[val_col][x[group_col] == groups[j]].values)) for i, j in zip(g1, g2)])

    return pd.DataFrame({
        'A': np.unique(data[group_col])[g1],
        'B': np.unique(data[group_col])[g2],
        'estimate': ef[:, 0],
        'magnitude': ef[:, 1]
    })


file = "prioritized.xlsx"
result_total = {}
sheets = [3, 5]
for j in range(len(sheets)):
    wb = xlrd.open_workbook(file)
    sheet = wb.sheet_by_index(sheets[j])

    results = {'Simulation runs': [], 'Total cost': [], 'Total waiting time': [], 'Maximum delay': [],
               'Average waiting time': [], 'Maximum capacity': [], 'CPU-time': []}

    for i in range(100):
        results['Simulation runs'].append(sheet.cell_value(i+1,0))
        results['Total cost'].append(sheet.cell_value(i+1,2))
        results['Total waiting time'].append(sheet.cell_value(i+1,3))
        results['Maximum delay'].append(sheet.cell_value(i+1,4))
        results['Average waiting time'].append(sheet.cell_value(i+1,5))
        results['Maximum capacity'].append(sheet.cell_value(i+1,6))
        results['CPU-time'].append(sheet.cell_value(i+1, 7))
    result_total[j] = results
print(VD_A(treatment=result_total[0]['Total cost'], control=result_total[1]['Total cost']))
print(VD_A(treatment=result_total[0]['Total waiting time'], control=result_total[1]['Total waiting time']))
print(VD_A(treatment=result_total[0]['Maximum delay'], control=result_total[1]['Maximum delay']))
print(VD_A(treatment=result_total[0]['Average waiting time'], control=result_total[1]['Average waiting time']))
print(VD_A(treatment=result_total[0]['Maximum capacity'], control=result_total[1]['Maximum capacity']))
