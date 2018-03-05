import pickle
import numpy as np
import sys

def get_col_filter(filename):
	col_filter = list()
	if '0' in filename:
		col_filter = [4, 5, 8, 10, 17]
	elif '1' in filename:
		col_filter = [4, 5, 8, 10, 15]
	elif '2' in filename:
		col_filter = [4, 8, 10, 29, 31]
	elif '3' in filename:
		col_filter = [4, 10, 23, 30, 31]
	else:
		print("Error: Robot must be one of 0, 1, 2, or 3")
		exit()
	return col_filter


if __name__ == "__main__":
	flag = sys.argv[1]
	filenames = []
	if flag == '--train':
		filenames = ['./train/label0.pkl','./train/label1.pkl','./train/label2.pkl','./train/label3.pkl']
	elif flag == '--test':
		filenames = ['./test/label0.pkl','./test/label1.pkl','./test/label2.pkl','./test/label3.pkl']
	else:
		print("Error: You must input --train or --test")
		exit()
	

	# Trim 4 files
	for file in filenames:
		col_filter = get_col_filter(file)
		# Read data
		with open(file,'rb') as f:
			mydata = pickle.load(f)
			arr = np.array(mydata)	
			arr = arr[:,col_filter]
			print(arr[:10])

		# Write data
		with open(file,'wb') as f:
			pickle.dump(arr, f)

	if flag == 'train':
		print("Trimming for train label has finished.\n")
	elif flag == 'test':
		print("Trimming for test label has finished.\n")
	else:
		print("Wrong flag")