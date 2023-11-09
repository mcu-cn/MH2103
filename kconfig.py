
import os
def fun(klipper_path):
    kpath = klipper_path + "/src/Kconfig"
    if os.path.exists(kpath):
        fin = open(kpath, "r")
    else :
        print("Kconfig file dir is error:" + kpath)
        return 1
    content = fin.read()
    fin.close()
    if "config MACH_STM32" not in content:
        print("No stm32 config, maybe Kconfig file is error.")
        return 1
    elif "config MACH_MH2" not in content:
        pos = content.find("config MACH_STM32")
        pos = content.find("endchoice",pos)
        content = content[:pos - 1] + \
                "\n\tconfig MACH_MH2\n\t\tbool \"MEGAHUNT mico MH2\"\n" + \
                content[pos:]
    if "source \"src/stm32/Kconfig\"" not in content:
        print("No source stm32 Kconfig")
    elif "source \"src/mh2/Kconfig\"" in content:
        print("MH2 Kconfig already source.")
    else :
        pos = content.find("src/stm32/Kconfig")
        pos = content.find("source",pos)
        content = content[:pos - 1] + \
                "\nsource \"src/mh2/Kconfig\"\n" + \
                content[pos:]
    fout = open(kpath,"w")
    fout.write(content)
    fout.close()
    return 0
