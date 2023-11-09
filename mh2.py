import sys
import os
import kconfig
import move
if len(sys.argv) == 1:
    print("you must assign KLIPPER dir\n")
    exit()
cur_path = os.path.dirname(os.path.abspath(__file__))
last_path = os.path.dirname(cur_path)
klipper_path = last_path + "/" + sys.argv[1]
# print("klipper-path:" + klipper_path)
if not os.path.exists(klipper_path):
    print("klipper directory not exists:" + klipper_path)
    exit()
kconfig.fun(klipper_path)
move.fun(klipper_path,cur_path)
print("SUCCESS!\nMH2103 is installed into " + sys.argv[1])
