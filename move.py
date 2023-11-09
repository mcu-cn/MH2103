import os
import sys
import shutil

def mv_file(old_path,new_path):
    # print("new path id:%d" % id(new_path))
    if not os.path.exists(new_path):
        # print("new path:" + new_path + " not exists so create it.")
        os.mkdir(new_path)
    for i in os.listdir(old_path):
        o_path = os.path.join(old_path,i)
        n_path = os.path.join(new_path,i)
        if os.path.isfile(o_path):
            # print("copy:" + o_path + " --> " + n_path)
            shutil.copy(o_path,n_path)
        if os.path.isdir(o_path):
            mv_file(o_path,n_path)

def fun(klipper_path,cur_path):
    old_path = cur_path + "/mh2"
    new_path = klipper_path + "/src/mh2/"
    # print("old-path:" + old_path)
    # print("new-path:" + new_path)
    mv_file(old_path,new_path)
    old_path = cur_path + "/lib"
    new_path = klipper_path + "/lib/mh2/"
    # print("old-path:" + old_path)
    # print("new-path:" + new_path)
    mv_file(old_path,new_path)
