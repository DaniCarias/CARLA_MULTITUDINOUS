import os

def delete_files_without_correspondent(list1, list2, list3):
    for filename in list1:
        # Check if the file has a correspondent in list2 and list3
        if all(any(abs(int(filename) - int(file)) <= 4 for file in sublist) for sublist in [list2, list3]):
            #and filename in list2 and filename in list3):
            continue
        else:
            print(f"Deleting file: {filename}")
            os.remove(filename)
            
            
if __name__ == "main":
    list1 = ["1233", "5678", "91011"]
    list2 = ["1234", "5678", "91012"]
    list3 = ["1233",         "91011"]

    delete_files_without_correspondent(list1, list2, list3)