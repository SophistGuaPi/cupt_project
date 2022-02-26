import time
import pandas
import pybullet as pb
import numpy as np
import pybullet_data
import os
import math
import random
import csv
import datetime
import gc

radius=0.133  #圆柱半径
d=0.1       #圆柱高
mass=0.1    #圆柱质量
objects=100  #一次实验投掷圆柱数量
nuitTime=1/240 #力学模拟单位时长，多少个单位秒进行一次力学模拟（不用改，这个数值是一点一点试出来的看的最舒服的）
minEnergy=1.4085499774960774*mass #停止实验时圆柱平均机械能（每次改变参数需要改，从打印的动能中选取）
restitutionAndFriction=[0.5,0.5,0.0005,0.002]#材质的物理属性，摩擦力，弹性等，其中第一个参数为弹力，第二个为横向摩擦力，第三个为转动摩擦力，第四个为滚动摩擦力


#生成Three-Sided Dice.obj模型文件以进行模拟实验
def create (r,d,n):  #三个参数分别为半径，高和模拟圆的多边形边的数量
    str1,str2=[],[]
    f=open("E:\project\cupt\model\Three-Sided Dice.txt","a")
    for i in range(n):
        angle=(2*math.pi/n)*i
        string="v "+str(round(r*math.cos(angle),6))+" "+str(round(r*math.sin(angle),6))+" "+str(d/2)+"\n"
        f.write(string)
    f.write("# UpSide point\n")
    for i in range(n):
        angle=(2*math.pi/n)*i
        string="v "+str(round(r*math.cos(angle),6))+" "+str(round(r*math.sin(angle),6))+" "+str(-d/2)+"\n"
        f.write(string)
    f.write("# DownSide point\n")
    for i in range(n):
        if(i==n-1):
            x=0
        else:
            x=i+1
        string="f "+str(i+1)+" "+str(n+i+1)+" "+str(n+x+1)+" "+str(x+1)+"\n"
        f.write(string)
    f.write("#SideSurface\n")
    for i in range(n):
        str1.append(" "+str(i+1))
        str2.append(" "+str(2*n-i))
    f.write("f"+"".join(str1)+"\n"+"# UpSideSurface"+"\n"+"f"+"".join(str2)+"\n"+"# DownSideSurface"+"\n")
    f.close()
    list=os.listdir("E:\project\cupt\model")
    os.chdir("E:\project\cupt\model")
    for i in list:
        if i[-4:]==".txt":
            os.rename(i,i.replace(".txt",".obj"))

def generateRandomQuaternion(): #生成随机四元数
    u1 = random.random()
    u2 = random.random()
    u3 = random.random()
    w = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
    x = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
    y = math.sqrt(u1) * math.sin(2 * math.pi * u3)
    z = math.sqrt(u1) * math.cos(2 * math.pi * u3)
    return (x, y, z, w)

def createDataFrame0():  #生成统计表格
    df=pandas.DataFrame(index=[n for n in range(1,objects+1)],
                        columns=["radius","thickness","mass","restitution", "lateralFriction",
                      "spinningFriction","rollingFriction","starPosition","starVelocity","side"])
    return df

def createDataFrame1(n):  #生成统计表格
    global count
    count=n
    print(f"-------进行{n}组实验--------")
    df=pandas.DataFrame(index=[i for i in range(1,count+1)],
                        columns=["radius","thickness","mass","restitution", "lateralFriction",
                      "spinningFriction","rollingFriction","count","upSideSurface amount","downSideSurface amount","sideSurface amount"])
    print("create the file...\nName of file is : three sided dice need parameters\n")
    return df


def EulerKineMaticaleQuationsTransform(uid):  #将惯性坐标系的角速度通过欧拉运动方程转化为非惯性参考系的角速度。
    EulerAngularVelocity=pb.getBaseVelocity(uid)[1]
    o2=pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(uid)[1])
    InertialAngularVelocity={}
    InertialAngularVelocity[0] = EulerAngularVelocity[0] * math.sin(o2[1] * math.pi / 180) * math.sin(
        o2[2] * math.pi / 180) + EulerAngularVelocity[1] * math.cos(o2[2] * math.pi / 180)
    InertialAngularVelocity[1] = EulerAngularVelocity[0] * math.sin(o2[1] * math.pi / 180) * math.cos(
        o2[2] * math.pi / 180) + EulerAngularVelocity[1] * math.sin(o2[2] * math.pi / 180)
    InertialAngularVelocity[2] = EulerAngularVelocity[0] * math.cos(o2[2] * math.pi / 180) + EulerAngularVelocity[2]
    return InertialAngularVelocity

#以上是函数声明

#---------------------------------------------------------------------------------------------------------------------------

#以下是代码入口
shift = [0, 0, 0]
scale = [1, 1, 1]
count=0
mode=False
num=0
SideSurface, UpSide, DownSide = 0, 0, 0
radiuslist=[]
dlist=[]
masslist=[]
restitutionAndFrictionlist=[]
objectslist=[]

#ps:记住这个路径，统计结果也会放在这个路径下。
if os.path.exists("E:\project\cupt\model\Three-Sided Dice.txt"):
    os.remove("E:\project\cupt\model\Three-Sided Dice.txt")
if os.path.exists("E:\project\cupt\model\Three-Sided Dice.obj"):
    os.remove("E:\project\cupt\model\Three-Sided Dice.obj")
create(radius,d,100)#(float(input("请输入半径r\n")),float(input("请输入厚度d\n")),int(input("请输入近似圆弧多边形边的数量n\n")))
#生成Three-Sided Dice.obj



#生成统计表格
while 1:
    i=int(input("统计模式:\n相同状态多骰子细节统计:1\n不同状态多骰子结果统计：2\n请输入统计编号:"))
    if(i==1):
        df=createDataFrame0()
        break
    else:                                   #第二种统计方法在这里改变每次实验的变量（注意控制变量）
        n=int(input("请输入实验组的数量:\n"))
        df=createDataFrame1(n)
        for i in range(n):
            radiuslist.append(0.1)
            dlist.append(i*0.01+0.1)
            masslist.append(0.1)
            restitutionAndFrictionlist.append([0.5,0.5,0.0005,0.002])
            objectslist.append(objects)
            mode=True
        break

#生成圆柱模型，并存入路径（需修改路径方能运行）
phy=pb.connect(pb.GUI)
print(pybullet_data.getDataPath())
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,0)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,0)
pb.configureDebugVisualizer(pb.COV_ENABLE_TINY_RENDERER,0)
pb.setRealTimeSimulation(0)
pb.setTimeStep(nuitTime)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
visual_shape_id = pb.createVisualShape(shapeType=pb.GEOM_MESH, fileName="Three-Sided Dice.obj",
                                       rgbaColor=[1, 1, 1, 1], specularColor=[0.4, 0.4, 0],
                                       visualFramePosition=shift, meshScale=scale)
collision_shape_id = pb.createCollisionShape(shapeType=pb.GEOM_CYLINDER, fileName="Three-Sided Dice.obj",
                                             collisionFramePosition=shift, meshScale=scale)

while 1:
    #链接物理引擎
    if mode==True:
        radius=radiuslist[num]
        d=dlist[num]
        mass=masslist[num]
        restitutionAndFriction=restitutionAndFrictionlist[num]
        objects=objectslist[num]

    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    pb.resetSimulation()
    pb.setGravity(0,0,-9.8)
    grand=pb.createCollisionShape(pb.GEOM_PLANE)
    pb.createMultiBody(0, grand)
    pb.changeDynamics(0, -1, restitution=restitutionAndFriction[0], lateralFriction=restitutionAndFriction[1],
                      spinningFriction=restitutionAndFriction[2], rollingFriction=restitutionAndFriction[3])
    pb.resetDebugVisualizerCamera(cameraDistance=8, cameraYaw=110, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
    visual_shape_id = pb.createVisualShape(shapeType=pb.GEOM_CYLINDER, length=d,radius=radius,
                                           rgbaColor=[1, 1, 1, 1], specularColor=[0.4, 0.4, 0],
                                           visualFramePosition=shift, meshScale=scale)
    collision_shape_id = pb.createCollisionShape(shapeType=pb.GEOM_CYLINDER, height=d,radius=radius,
                                                 collisionFramePosition=shift, meshScale=scale)

    #在物理引擎中添加圆柱模型
    pb.setAdditionalSearchPath("E:\project\cupt\model")
    for i in range(1,objects+1):
        position=[0,0,random.uniform(4,6)]
        orientation=generateRandomQuaternion()
        #（作用时间是一个单位模拟时间，大概吧...找不到资料...）
        velocity=[[
            (random.uniform(-1,1) * 3),
            (random.uniform(-1,1) * 3),
            (random.uniform(-1,1) * 3)],

            [(random.uniform(-1, 1) * 30),
             (random.uniform(-1, 1) * 30),
             (random.uniform(-1, 1) * 30)]
        ]

        if mode==False:
            df.loc[i]["starPosition"] = list(np.round(np.array(position), 2))

        x=pb.createMultiBody(baseMass=mass,baseCollisionShapeIndex=collision_shape_id,
                             baseVisualShapeIndex=visual_shape_id,basePosition=position,
                             baseOrientation=orientation,useMaximalCoordinates=True,flags=pb.URDF_IGNORE_COLLISION_SHAPES)
        pb.changeDynamics(x,-1, restitution=restitutionAndFriction[0], lateralFriction=restitutionAndFriction[1],
                          spinningFriction=restitutionAndFriction[2],rollingFriction=restitutionAndFriction[3])
        pb.setCollisionFilterGroupMask(x, -1, collisionFilterGroup=-1, collisionFilterMask=0)
        pb.resetBaseVelocity(x,velocity[0],velocity[1])

    #开始显示可视化窗口
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,1)

    # while True:
    #     pb.stepSimulation()
    #     time.sleep(1/240)
    #     if (input("输入任意字符以暂停运行")):
    #         break
    #模拟开始
    while True:
        mechanicalEnergy=0
        velocityAndAngularVelocity={}
        velocity=[]
        angularVelocity=[]

        #进行一步模拟
        pb.stepSimulation()
        # time.sleep(1/240)

        #统计
        for i in range(1,objects+1):
            velocity.append(pb.getBaseVelocity(i)[0])
            angularVelocity.append(EulerKineMaticaleQuationsTransform(i))
            velocityAndAngularVelocity[i]=[pb.getBaseVelocity(i)[0],pb.getBaseVelocity(i)[1]]
            mechanicalEnergy=mechanicalEnergy+\
                             (mass*(velocityAndAngularVelocity[i][0][0]**2+velocityAndAngularVelocity[i][0][1]**2+
                                    velocityAndAngularVelocity[i][0][2]**2))/2\
                             +np.sqrt(
                                 ((((mass*radius**2)/2)*velocityAndAngularVelocity[i][1][2]**2)/2)**2
                                 +((((mass*(3*radius**2+(d/2)**2))/12)*velocityAndAngularVelocity[i][1][0]**2)/2)**2
                                 +((((mass*(3*radius**2+(d/2)**2))/12)*velocityAndAngularVelocity[i][1][1]**2)/2)**2
                             )\
                             +mass*9.8*pb.getBasePositionAndOrientation(i)[0][2]
        mechanicalEnergy=mechanicalEnergy/objects
        print(mechanicalEnergy)
        if (mechanicalEnergy<=minEnergy):
            for i in range(1,objects+1):
                pos, angle = pb.getBasePositionAndOrientation(i)
                angle = pb.getEulerFromQuaternion(angle)
                angle = (angle[0] + 2 * math.pi) % (2 * math.pi)

                #以下公式是从网上抄来的，未经证明，是理论证明的重点之一
                if math.pi / 2 - (math.pi / 2 - 1.3) <= angle <= math.pi / 2 + (math.pi / 2 - 1.3) or math.pi / 2 * 3 - (
                        math.pi / 2 - 1.3) <= angle <= math.pi / 2 * 3 + (math.pi / 2 - 1.3):
                    if mode==False:
                        df.loc[i]["side"] = "SideSurface"
                    SideSurface+=1
                if angle <= 1.3 or 2 * math.pi - 1.3 <= angle:
                    if mode == False:
                        df.loc[i]["side"] = "UpSide"
                    UpSide+=1
                if math.pi - 1.3 <= angle <= math.pi + 1.3:
                    if mode == False:
                        df.loc[i]["side"] = "DownSide"
                    DownSide+=1
            if mode==False:
                for i in range(1,objects+1):
                    df.loc[i]["radius"]=radius
                    df.loc[i]["thickness"]=d
                    df.loc[i]["mass"]=mass
                    df.loc[i]["starVelocity"] =list(np.round(pb.getBaseVelocity(i)[0],2)),list(np.round(pb.getBaseVelocity(i)[1],2))
                    df.loc[i]["restitution"] = restitutionAndFriction[0]
                    df.loc[i]["lateralFriction"] = restitutionAndFriction[1]
                    df.loc[i]["spinningFriction"] = restitutionAndFriction[2]
                    df.loc[i]["rollingFriction"] = restitutionAndFriction[3]
                break
            else:
                df.loc[num+1]["radius"] = radius
                df.loc[num+1]["thickness"] = d
                df.loc[num+1]["mass"] = mass
                df.loc[num+1]["restitution"] = restitutionAndFriction[0]
                df.loc[num+1]["lateralFriction"] = restitutionAndFriction[1]
                df.loc[num+1]["spinningFriction"] = restitutionAndFriction[2]
                df.loc[num+1]["rollingFriction"] = restitutionAndFriction[3]
                df.loc[num+1]["count"] = objects
                df.loc[num+1]["upSideSurface amount"] = UpSide
                df.loc[num+1]["downSideSurface amount"] = DownSide
                df.loc[num+1]["sideSurface amount"] = SideSurface
                print(f"-----第{num + 1}组实验结束-----\n")
                print(df)
                num += 1
                SideSurface, UpSide, DownSide = 0, 0, 0
                break

    if mode == False:
        break
    elif num >= count:
        break



pb.disconnect()
print(df)
currenttime = datetime.datetime.now().replace(microsecond=0)
currenttime = currenttime.strftime("%Y-%m-%d,%H-%M-%S" + ".csv")
print("Writing data to file...\nName of file is :"+ "result_of three sided dice " + currenttime +"\n")
df.to_csv("result_of three sided dice " + currenttime)
