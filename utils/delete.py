import sys
import argparse

parser = argparse.ArgumentParser(description='Script to delete some folders.')
parser.add_argument("type", type=str, default="dataset")


if __name__ == "__main__":
    
    args = parser.parse_args()
    delete_type = args.type

    if delete_type == "dataset":
        exec(open('./deletes/delete_dataset.py').read())
    elif delete_type == "ground_truth":
        exec(open('./deletes/delete_ground_truth.py').read())
    elif delete_type == "out":
        exec(open('./deletes/delete_out.py').read())
    elif delete_type == "segm":
        exec(open('./deletes/delete_lidarSegm.py').read())
    else:
        print("Invalid type")
        sys.exit(1)