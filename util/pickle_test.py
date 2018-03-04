import os
import pickle


PICKLE_FILE = 'pickle.pkl'


def main():
    # append data to the pickle file
    add_to_pickle(PICKLE_FILE, [1,2,3])
    add_to_pickle(PICKLE_FILE, [4,5,6])

    # load & show all stored objects
    for item in read_from_pickle(PICKLE_FILE):
        print(item)
    os.remove(PICKLE_FILE)

def add_to_pickle(path, item):
    with open(path, 'ab') as file:
        pickle.dump(item, file, pickle.HIGHEST_PROTOCOL)


def read_from_pickle(path):
    with open(path, 'rb') as file:
        try:
            while True:
                yield pickle.load(file)
        except EOFError:
            pass

if __name__ == '__main__':
    main()
