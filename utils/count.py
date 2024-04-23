import sys
import argparse

parser = argparse.ArgumentParser(description='Script to count some folders.')
parser.add_argument("type", type=str, default="dataset")

if __name__ == "__main__":
    
    args = parser.parse_args()
    count_type = args.type
    
    if count_type == "dataset":
        exec(open('./counts/count_dataset.py').read())
    elif count_type == "segm":
        exec(open('./counts/count_lidarSegm.py').read())
    elif count_type == "out":
        exec(open('./counts/count_out.py').read())
    else:
        print("Invalid type")
        sys.exit(1)