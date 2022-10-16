import csv
import matplotlib.pyplot as plt


data = [[] for _ in range(8)]
with open('length.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        for count in range(8):
            data[count].append(row[count])
print(data)

fig = plt.figure()
 
# Creating axes instance
ax = fig.add_axes([0,0,0,0,1,1,1,1])
 
# Creating plot
bp = ax.boxplot(data)
 
# show plot
plt.show()

