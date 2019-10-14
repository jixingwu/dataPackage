import csv
import sys
import numpy as np

def readFile(path):
    f = open(path)
    first_ele = True
    for data in f.readlines():
        data = data.strip('\n')
        nums = data.split(" ")
        if first_ele:
            nums = [int(x) for x in nums ]
            matrix = np.array(nums)
            first_ele = False
        else:
            nums = [int(x) for x in nums]
            matrix = np.c_[matrix,nums]
    f.close()
    return matrix

def main(argv):

    csvPath = sys.argv[1] + "/LogFiles_tmp/GroundTruthAGM.csv"
    csvFile = open(csvPath, 'r')
    gtaMat = readFile(csvPath)
    gta = csv.reader(csvFile)

    csvPath = sys.argv[1] + "/LogFiles_tmp/GroundTruthAGL.csv"
    csvFile = open(csvPath, 'r')
    gtlMat = readFile(csvPath)
    gtl = csv.reader(csvFile)

    csvWriter = open(sys.argv[2], 'w', newline='')
    fileheader = ["timestamp", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"]
    data = []



    # for gta_data, gtl_data in zip(gta, gtl):
    #     csvWriter.writerow(fileheader)
    #     data.append(gta_data[0])








    csvWriter.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
