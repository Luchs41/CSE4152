import numpy as np
import matplotlib.pyplot as plt
def display(input_txt_path, output_txt_path):
    
    
    '''
    input1 : input_txt_path = path of input_example.txt
    input2 : output_txt_path = path of output_example.txt
    return : save convex_hull image
    '''
    
    with open(input_txt_path, "r") as f:
        input_lines = f.readlines()
    with open(output_txt_path, "r") as f:
        output_lines = f.readlines()
        
    whole_points = input_lines
    area = round(float(output_lines[0]), 1)
    hull_points = output_lines[1:]

    x_list = [int(x.split(" ")[0]) for x in whole_points]
    y_list = [int(x.split(" ")[1]) for x in whole_points]
    plt.plot(x_list, y_list, marker='.', linestyle='None')

    hx_list = [int(x.split(" ")[0]) for x in hull_points]
    hy_list = [int(x.split(" ")[1]) for x in hull_points]

    plt.plot(hx_list, hy_list, marker='*', linestyle='None', markersize=10)

    title = plt.title(f'Area : {area}')
    plt.setp(title, color='r')
    plt.savefig(output_txt_path[:-3]+"png", bbox_inches='tight')

def Area(points) : 
    area = 0
    n = len(points)
    x, y = points[0][0], points[0][1]
    points.append([x, y])
    # print(points)
    for i in range(n) : 
        area += (points[i][0] * points[i + 1][1]) - (points[i + 1][0] * points[i][1])
    points.pop()
    return round(abs(area)/2, 1)

# ccw : returns
def ccw(p1, p2, p3) : 
    return p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
def ConvexHull(points) : 
    sortedPoints = sorted(points)
    lower = list()
    for p in sortedPoints : 
        while len(lower) >= 2 and ccw(lower[-2], lower[-1], p) <= 0 : 
            lower.pop()
        lower.append(p)
    
    upper = list()
    for p in reversed(sortedPoints) : 
        while len(upper) >= 2 and ccw(upper[-2], upper[-1], p) <= 0 : 
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def inpoly(polygon, point) : 
    ccw_list = list()
    for x in range(len(polygon)) : 
        ccw_list.append(ccw(point, polygon[x], polygon[x - 1]))
    result = 1
    for i in range(len(ccw_list)) : 
        if ccw_list[i] > 0 : 
            result = 0
    return result



if __name__ == "__main__" : 
    # input points
    input1 = open("input1.txt", "r")
    input2 = open("input2.txt", "r")
    input3 = open("input3.txt", "r")
    points1 = list()
    points2 = list()
    points3 = list()
    while 1 : 
        indata = input1.readline()
        if indata == '' : 
            break
        inx, iny = map(int, indata.strip().split())
        points1.append([inx, iny])
    while 1 : 
        indata = input2.readline()
        if indata == '' : 
            break
        inx, iny = map(int, indata.strip().split())
        points2.append([inx, iny])
    while 1 : 
        indata = input3.readline()
        if indata == '' : 
            break
        inx, iny = map(int, indata.strip().split())
        points3.append([inx, iny])
    input1.close()
    input2.close()
    input3.close()
    # input end
    
    convex1 = ConvexHull(points1)
    convex2 = ConvexHull(points2)
    convex3 = ConvexHull(points3)
    
    
    Area1 = Area(convex1)
    Area2 = Area(convex2)
    Area3 = Area(convex3)
    
    input_polygon = open("point_in_polygon_input.txt", "r")
    check_list = list()
    while 1 : 
        indata = input_polygon.readline()
        if indata == '' : 
            break
        inx, iny = map(int, indata.strip().split())
        check_list.append([inx, iny])
    
    input_polygon.close()
    inpoly1 = list()
    inpoly2 = list()
    inpoly3 = list()
    
    for i in check_list : 
        inpoly1.append(inpoly(convex1, i))
    
    for i in check_list : 
        inpoly2.append(inpoly(convex2, i))
    
    for i in check_list : 
        inpoly3.append(inpoly(convex3, i))
    # output
    output1 = open("이건영_output1.txt", "w")
    output1.write(str(Area1) + "\n")
    for i in convex1 : 
        x, y = i
        output1.write("{} {}\n".format(x, y))
    output1.close()
    display("./input1.txt", "./이건영_output1.txt")
    plt.cla()
    polyout1 = open("이건영_point_in_polygon_output1.txt", "w")
    for i in inpoly1 : 
        if i == 0 : 
            polyout1.write("out\n")
        else : 
            polyout1.write("in\n")
    polyout1.close()

    output2 = open("이건영_output2.txt", "w")
    output2.write(str(Area2) + "\n")
    for i in convex2 : 
        x, y = i
        output2.write("{} {}\n".format(x, y))
    output2.close()
    display("./input2.txt", "./이건영_output2.txt")
    plt.cla()
    polyout2 = open("이건영_point_in_polygon_output2.txt", "w")
    for i in inpoly2 : 
        if i == 0 : 
            polyout2.write("out\n")
        else : 
            polyout2.write("in\n")
    polyout2.close()

    output3 = open("이건영_output3.txt", "w")
    output3.write(str(Area3) + "\n")
    for i in convex3 : 
        x, y = i
        output3.write("{} {}\n".format(x, y))
    output3.close()
    display("./input3.txt", "./이건영_output3.txt")
    plt.cla()
    polyout3 = open("이건영_point_in_polygon_output3.txt", "w")
    for i in inpoly3 : 
        if i == 0 : 
            polyout3.write("out\n")
        else : 
            polyout3.write("in\n")
    polyout3.close()