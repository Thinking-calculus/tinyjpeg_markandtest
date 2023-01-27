import os
import sys


def seperate_yuv_420(file_name, width, hight):
    out_bin_file_name = []

    pixel_full = int(width)*int(hight)

    y = pixel_full
    u = int(pixel_full/4)
    v = int(pixel_full/4)
    y_start = 0
    y_end = y

    u_start = y_end
    u_end = u+u_start

    v_start = u_end
    v_end = v_start+v

    # print("The y ="+str(y)+" u_start="+str(u_start)+";u_end="+ str(u_end)+ ";v_start="+str(v_start))

    with open(file_name, "rb") as f:
        out_bin_file_name = f.read()

        # y file
        y_file = open(".y", "ba")
        y_file.write(out_bin_file_name[y_start:y_end])
        y_file.close()

        # u file
        u_file = open(".u", "ba")
        u_file.write(out_bin_file_name[u_start:u_end])
        u_file.close()

        # v file
        u_file = open(".v", "ba")
        u_file.write(out_bin_file_name[v_start:v_end])
        u_file.close()

        # print(out_bin_file_name[8384:8385])
    ...


def seperate_yuv(file_name, width, hight, yuv_type):
    if (os.path.exists(file_name) == False):
        print("pls make sure have enought file")
        return 0
    elif (isinstance(width, int) and isinstance(hight, int)):
        print("pls make sure input file scale is reasonable")
        return 0
    elif (yuv_type == "yuv444" and yuv_type == "yuv422" and yuv_type == "yuv420"):
        print("pls make sure the input file's sample type is reasonable")
        return 0

    # delete old file
    # for root, dirs, files in os.walk("."):
    #     for name in files:
    #         if name.startswith("."): #only delete the file that filename start with "."
    #             os.remove(os.path.join(root, name))

    if (yuv_type == "yuv420"):
        # todo
        seperate_yuv_420(file_name, width, hight)
        ...
    else:
        print("sorry,"+yuv_type+" file type still didn't support")


def combine_yuv(y_file, u_file, v_file, output_file="new.yuv"):
    try:
        os.remove(output_file)  # delete old file
    except:
        pass

    if ((os.path.exists(y_file) and os.path.exists(u_file) and os.path.exists(v_file)) == False):
        print("pls make sure have enought file")
        return 0

    y_file_obj = open(y_file, "br+")
    y_data = y_file_obj.read()
    y_file_obj.close()

    u_file_obj = open(u_file, "br+")
    u_data = u_file_obj.read()
    u_file_obj.close()

    v_file_obj = open(v_file, "br+")
    v_data = v_file_obj.read()
    v_file_obj.close()

    output_file = open(output_file, "ba")
    output_file.write(y_data+u_data+v_data)
    output_file.close()


if __name__ == '__main__':

    input_err_flag = 0

    if (len(sys.argv) == 1):
        # nothing input
        print(
            "-s [yuv file] [file width] [file hight] [yuv type{yuv444,yuv422,yuv420}]")
        print(
            "-c [input luma file{y}] [input luma file{u}] [input luma file{u}] [output file name]")
    else:
        if (sys.argv[1] == "-s"):
            if (len(sys.argv) != 6):
                input_err_flag = 1
            else:
                seperate_yuv(sys.argv[2],
                             sys.argv[3], sys.argv[4], sys.argv[5],)
        elif (sys.argv[1] == "-c"):
            if (len(sys.argv) != 6):
                input_err_flag = 1
            else:
                combine_yuv(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        else:
            print("1 Input err,pls make sure")

    if (input_err_flag):
        print("2 Input err,pls make sure")

    # input_y_file = "test.Y"
    # input_u_file = "test.U"
    # input_v_file = "test.V"

    # bin_file_name = "origin.yuv"
    # file_width = 160
    # file_hight = 160
    # yuv = 1  # 1:yuv420; 2:yuv422 3:yuv444

    # seperate_yuv(bin_file_name, file_width, file_hight, yuv)
    # combine_yuv(input_y_file,input_u_file,input_v_file)
    # todo
