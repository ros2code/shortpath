#
#    File: matrixTest.py
# 
#    Description: Test cases for shortestPathFinder.py
#
#    Input: Matrix dimensions, path and data in json format
#    Output: Checks if the expected shortest path is returned
#

import os
import json
import time
from datetime import datetime
from shortestPathFinder import MatrixPath
from copy import copy, deepcopy


class MatrixTest():
    testlist_files = []
    testlist_count = 0

    def __init__(self, folder_path):
        self.folder_path = folder_path

    def build_testlist(self):
        """ Build the json file list in the specified folder path """
        for file in os.listdir(self.folder_path):
            if file.endswith(".json"):
                self.testlist_files.append(os.path.join(self.folder_path, file))
        self.testlist_count = len(self.testlist_files)


    def run(self):
        self.build_testlist()
        if self.testlist_count == 0:
            print("No tests found in folder %s. Exiting test run"%(self.folder_path))
            exit()
        else:
            print("Found %d test files in %s folder"%(self.testlist_count, self.folder_path))

            for i in range(self.testlist_count):
                print("\n\n------------------------------------------------------------------\n")
                print("Matrix %s" % os.path.basename(self.testlist_files[i]))
                with open(self.testlist_files[i]) as f:
                    jdata = json.load(f)

                    mxndata = jdata["data"]
                    rows = len(mxndata)
                    cols = len(mxndata[0])

                    # Check if the matrix data dimensions match the stored dimensions
                    if ((rows != jdata["dimensions"]["rows"]) or (cols != jdata["dimensions"]["cols"])):
                        print("Size of data is not same as stored dimensions")
                        exit()
                    
                    print("Dimensions %dx%d" % (rows, cols))

                    paths = jdata["paths"]
                    matrix_instance = MatrixPath (mxndata)
                    
                    for j in range(len(paths)):
                        shortpath = []
                        start = (paths[j]["start"][0],paths[j]["start"][1])
                        end = (paths[j]["end"][0],paths[j]["end"][1])
                        print("\n\nFinding path for %s -> %s" % (paths[j]["start"], paths[j]["end"]))

                        begin_time = datetime.now()
                        shortpath = matrix_instance.findpath(start, end)
                        end_time = datetime.now()
                        print(shortpath)

                        if (shortpath):
                            matrix_data = deepcopy(mxndata)
                            matrix_instance.print_matrix(matrix_data, shortpath)

                        str1 = str(shortpath)
                        str1 = str1.replace(" ","")
                        str2 = paths[j]["expected"]
                        str2 = str2.replace(" ","")   
                        
                        if (str1 == str2):
                            print("Success: Found path is same as expected.")
        
                        else:
                            print("Eror: Path is not same as expected.")

                        print("Time take for finding path: ",(end_time-begin_time))

                        
                        
if __name__ == '__main__':
    testrun = MatrixTest(folder_path = "tests")
    testrun.run()