import csv
import matplotlib.pyplot as plt

csv_col = 1
def length_plot(name):
    data = [[] for _ in range(csv_col)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(csv_col):
                data[count].append(float(row[count]))
    print(data)

    # fig = plt.figure()
    
    # Creating axes instance
    fig1, ax1 = plt.subplots()
    ax1.set_title('Length Variation')
    ax1.boxplot(data)
    ax1.set_xlabel("RRT")
    ax1.set_ylabel("Length of path")
    
    # show plot
    plt.show()

def time_plot(name):
    data = [[] for _ in range(csv_col)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(csv_col):
                data[count].append(float(row[count])/10**6)
    print(data)

    # fig = plt.figure()
    
    # Creating axes instance
    fig1, ax1 = plt.subplots()
    ax1.set_title('Run Time Variation')
    ax1.boxplot(data)
    ax1.set_xlabel("RRT")
    ax1.set_ylabel("Time taken to run(s)")
    
    # show plot
    plt.show()

def valid_plot(name):
    data = [0 for _ in range(csv_col)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(csv_col):
                data[count] += int(row[count])
    print(data)

    fig = plt.figure()
    
    # Creating axes instance
    plt.bar(0,data)
    # fig1, ax1 = plt.subplots()
    plt.title('Run Time Variation')
    plt.xlabel("RRT")
    plt.ylabel("Number of valid solutions")
    
    # show plot
    plt.show()

length_plot('length_rrt.csv')
time_plot('time_rrt.csv')
valid_plot('valid_rrt.csv')

length_plot('length_rrt_w1.csv')
time_plot('time_rrt_w1.csv')
valid_plot('valid_rrt_w1.csv')

length_plot('length_rrt_w2.csv')
time_plot('time_rrt_w2.csv')
valid_plot('valid_rrt_w2.csv')