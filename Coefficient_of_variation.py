import numpy as np
import xlrd
import matplotlib.pyplot as plt
import matplotlib2tikz


string = 'CPU-time'
file = "distributed.xlsx"


wb = xlrd.open_workbook(file)
sheet = wb.sheet_by_index(0)

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

# c_v_lst = []
# for j in range(100):
#     mean = round(np.mean(np.array(results[string][:j+1])), 3)
#     sigma = round(np.sqrt(np.var(np.array(results[string][:j+1]))), 3)
#     c_v_lst.append(sigma/mean)
#
# plt.plot(results['Simulation runs'], c_v_lst)
# plt.xlabel('Simulation run')
# if string != 'CPU-time':
#     string = string.lower()
# plt.ylabel('Coefficient of variation of '+string)
# plt.grid()
# plt.show()

c_v_lst_total_cost = []
c_v_lst_maximum_delay = []
c_v_lst_total_waiting_time = []
for j in range(100):
    mean_total_cost = round(np.mean(np.array(results['Total cost'][:j+1])), 3)
    sigma_total_cost = round(np.sqrt(np.var(np.array(results['Total cost'][:j+1]))), 3)
    c_v_lst_total_cost.append(sigma_total_cost/mean_total_cost)

    mean_maximum_delay = round(np.mean(np.array(results['Maximum delay'][:j+1])), 3)
    sigma_maximum_delay = round(np.sqrt(np.var(np.array(results['Maximum delay'][:j+1]))), 3)
    c_v_lst_maximum_delay.append(sigma_maximum_delay/mean_maximum_delay)

    mean_total_waiting_time = round(np.mean(np.array(results['Total waiting time'][:j+1])), 3)
    sigma_total_waiting_time = round(np.sqrt(np.var(np.array(results['Total waiting time'][:j+1]))), 3)
    c_v_lst_total_waiting_time.append(sigma_total_waiting_time/mean_total_waiting_time)

plt.plot(results['Simulation runs'], c_v_lst_total_cost, label='Total cost')
plt.plot(results['Simulation runs'], c_v_lst_maximum_delay, label='Maximum delay')
plt.plot(results['Simulation runs'], c_v_lst_total_waiting_time, label='Total waiting time')
plt.xlabel('Simulation run')
if string != 'CPU-time':
    string = string.lower()
plt.ylabel('Coefficient of variation')
plt.legend(loc='lower right')
plt.grid()
matplotlib2tikz.save('CoV-Distributed.tex')
plt.show()