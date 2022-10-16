import csv
import matplotlib.pyplot as plt


def length_plot(name):
    data = [[] for _ in range(8)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(8):
                data[count].append(float(row[count]))
    print(data)

    # fig = plt.figure()
    
    # Creating axes instance
    fig1, ax1 = plt.subplots()
    ax1.set_title('Length Variation')
    ax1.boxplot(data)
    ax1.set_xlabel("Node count, connection distance")
    ax1.set_ylabel("Length of path")
    
    # show plot
    plt.show()

def time_plot(name):
    data = [[] for _ in range(8)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(8):
                data[count].append(float(row[count])/10**6)
    print(data)

    # fig = plt.figure()
    
    # Creating axes instance
    fig1, ax1 = plt.subplots()
    ax1.set_title('Run Time Variation')
    ax1.boxplot(data)
    ax1.set_xlabel("Node count, connection distance")
    ax1.set_ylabel("Time taken to run(s)")
    
    # show plot
    plt.show()

def valid_plot(name):
    data = [0 for _ in range(8)]
    with open(name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            for count in range(8):
                data[count] += int(row[count])
    print(data)

    fig = plt.figure()
    
    # Creating axes instance
    plt.bar(range(1,9),data)
    # fig1, ax1 = plt.subplots()
    plt.title('Run Time Variation')
    plt.xlabel("Node count, connection distance")
    plt.ylabel("Number of valid solutions")
    
    # show plot
    plt.show()

length_plot('length.csv')
time_plot('time.csv')
valid_plot('valid.csv')