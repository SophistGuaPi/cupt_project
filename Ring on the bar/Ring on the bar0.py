import time
import pandas
import pybullet as pb
import numpy as np
import pybullet_data
import os
import math
import random
import datetime

mass=1
nuitTime=1/240
radiusOfRing_Ring=0.4
radiusOfRing_section=0.03
radiusOfRing_bar=0.2
n0=20

def createRing(r0,r1,n0,n1):
    str1, str2 = [], []
    f = open("E:\project\cupt\model\Ring On The Bar_Ring.txt", "a")
    for i in range(n0):
        angle0=(2*math.pi/n0)*i
        for j in range(n1):
            angle1=(2*math.pi/n1)*j
            string="v "+str(round((r0+r1-r1*math.cos(angle1))*math.cos(angle0),6))+" "\
                   +str(round((r0+r1-r1*math.cos(angle1))*math.sin(angle0),6))+" "+str(round(r1*math.sin(angle1),6))+"\n"
            f.write(string)
    f.write("# all point\n")
    arr=[[0]*n1 for _ in range(n0)]
    for i in range(n0):
        for j in range(n1):
            arr[i][j]=i*n1+j+1
    i,j,k=0,0,0
    while True:
        if k >=n0*n1:
            break
        if (j>=n0):
            j=0
        if (i>=n1):
            i=0
            j+=1
        if (i==n1-1):
            x=-1
        else:
            x=i
        if (j == n0 - 1):
            y = -1
        else:
            y = j
        string = "f " + str(arr[j][i]) + " " + str(arr[j][x+1]) + " " + str(arr[y+1][x+1]) + " " + str(arr[y+1][i]) + "\n"
        i+=1
        k+=1
        f.write(string)
    f.write("# all face\n")
    f.close()
    list = os.listdir("E:\project\cupt\model")
    os.chdir("E:\project\cupt\model")
    for i in list:
        if i[-4:]==".txt":
            os.rename(i,i.replace(".txt",".obj"))

def createBar (r,d,n):  #三个参数分别为半径，高和模拟圆的多边形边的数量
    str1,str2=[],[]
    f=open("E:\project\cupt\model\Ring On The Bar_Bar.txt","a")
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

def createRing_nuit (r0,r1,n0,n1):  #三个参数分别为半径，高和模拟圆的多边形边的数量
    str1,str2=[],[]
    d=2*r0*math.sin(math.pi/n0)
    f=open("E:\project\cupt\model\Ring On The Bar_RING_nuit.txt","a")
    for i in range(n1):
        angle=(2*math.pi/n1)*i
        string="v "+str(d/2)+" "+str(round(r1*math.cos(angle),6))+" "+str(round(r1*math.sin(angle),6))+"\n"
        f.write(string)
    f.write("# UpSide point\n")
    for i in range(n1):
        angle=(2*math.pi/n1)*i
        string="v "+str(-d/2)+" "+str(round(r1*math.cos(angle),6))+" "+str(round(r1*math.sin(angle),6))+"\n"
        f.write(string)
    f.write("# DownSide point\n")
    for i in range(n1):
        if(i==n1-1):
            x=0
        else:
            x=i+1
        string="f "+str(i+1)+" "+str(n1+i+1)+" "+str(n1+x+1)+" "+str(x+1)+"\n"
        f.write(string)
    f.write("#SideSurface\n")
    for i in range(n1):
        str1.append(" "+str(i+1))
        str2.append(" "+str(2*n1-i))
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

def EulerKineMaticaleQuationsTransform(uid):  #将惯性坐标系的角速度通过欧拉运动方程转化为非惯性参考系的角速度。
    EulerAngularVelocity=pb.getBaseVelocity(uid)[1]
    o2=pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(uid)[1])
    InertialAngularVelocity=[]
    InertialAngularVelocity.append(EulerAngularVelocity[0] * math.sin(o2[1] * math.pi / 180) * math.sin(
        o2[2] * math.pi / 180) + EulerAngularVelocity[1] * math.cos(o2[2] * math.pi / 180))
    InertialAngularVelocity.append(EulerAngularVelocity[0] * math.sin(o2[1] * math.pi / 180) * math.cos(
        o2[2] * math.pi / 180) + EulerAngularVelocity[1] * math.sin(o2[2] * math.pi / 180))
    InertialAngularVelocity.append(EulerAngularVelocity[0] * math.cos(o2[2] * math.pi / 180) + EulerAngularVelocity[2])
    return InertialAngularVelocity

def createDataFrame(n):  #生成统计表格
    print(f"---第{n}次实验的数据汇总---\n")
    df=pandas.DataFrame(index=[i for i in range(1000)],columns=["radiusOfRing_Ring","radiusOfRing_section","radiusOfRing_bar","mass","restitution", "lateralFriction",
                      "spinningFriction","rollingFriction","position","velocity","angularVelocity","time"])
    print(df)
    return df

# -----------------------------------------------------------------------------------#

if os.path.exists("E:\project\cupt\model\Ring On The Bar_Ring_nuit.txt"):
    os.remove("E:\project\cupt\model\Ring On The Bar_Ring_nuit.txt")
if os.path.exists("E:\project\cupt\model\Ring On The Bar_Ring_nuit.obj"):
    os.remove("E:\project\cupt\model\Ring On The Bar_Ring_nuit.obj")
if os.path.exists("E:\project\cupt\model\Ring On The Bar_Bar.txt"):
    os.remove("E:\project\cupt\model\Ring On The Bar_Bar.txt")
if os.path.exists("E:\project\cupt\model\Ring On The Bar_Bar.obj"):
    os.remove("E:\project\cupt\model\Ring On The Bar_Bar.obj")
createRing_nuit(radiusOfRing_Ring,radiusOfRing_section,n0,100)
createBar(radiusOfRing_bar,1000,100)
df=createDataFrame(1)

shift = [0, 0, 0]
scale = [1, 1, 1]
phy=pb.connect(pb.GUI)
pb.setTimeStep(nuitTime)
print(pybullet_data.getDataPath())
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
grand=pb.createCollisionShape(pb.GEOM_PLANE)
pb.createMultiBody(0, grand)
pb.setGravity(0,0,-9.8)
pb.setRealTimeSimulation(0)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,0)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,0)
pb.configureDebugVisualizer(pb.COV_ENABLE_TINY_RENDERER,0)


collision_shape_id_0=pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName="Ring On The Bar_Ring_nuit.obj",
                                           collisionFramePosition=shift,meshScale=scale)
visual_shape_id_1=pb.createVisualShape(shapeType=pb.GEOM_MESH,fileName="Ring On The Bar_Bar.obj",
                                           rgbaColor=[1,1,1,1],specularColor=[0.4,0.4,0],visualFramePosition=shift,meshScale=scale)
collision_shape_id_1=pb.createCollisionShape(shapeType=pb.GEOM_MESH,fileName="Ring On The Bar_Bar.obj",
                                           collisionFramePosition=shift,meshScale=scale)
pb.setAdditionalSearchPath("E:\project\cupt\model")
pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=collision_shape_id_1,
                         baseVisualShapeIndex=visual_shape_id_1,basePosition=[0,0,500],
                         baseOrientation=[0,0,0,1],useMaximalCoordinates=True)
barID=pb.createMultiBody(baseCollisionShapeIndex=collision_shape_id_1,baseVisualShapeIndex=visual_shape_id_1,useMaximalCoordinates=True)

#生成环的代码，前面已经生成体积元collision_shape_id_0
link_Masses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
indices = []
jointTypes = []
axis = []
for i in range(n0):
    link_Masses.append(mass/n0)
    linkCollisionShapeIndices.append(collision_shape_id_0)
    linkVisualShapeIndices.append(-1)
    linkPositions.append([(radiusOfRing_section+radiusOfRing_Ring)*math.sin(2*math.pi/n0),(radiusOfRing_section+radiusOfRing_Ring)*(1-math.cos(2*math.pi/n0)),0])
    linkOrientations.append(pb.getQuaternionFromEuler([0,0,2*math.pi/n0]))
    linkInertialFramePositions.append([(radiusOfRing_section+radiusOfRing_Ring)*math.sin(2*math.pi/n0),(radiusOfRing_section+radiusOfRing_Ring)*(1-math.cos(2*math.pi/n0)),0])
    linkInertialFrameOrientations.append(pb.getQuaternionFromEuler([0,0,2*math.pi/n0]))
    indices.append(i)
    jointTypes.append(pb.JOINT_FIXED)
    axis.append([0, 0, 1])
ringID=pb.createMultiBody(mass/n0,collision_shape_id_0,-1,
                              [0,-radiusOfRing_Ring,100],
                              [0,0,0,1],
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkOrientations=linkOrientations,
                              linkPositions=linkPositions,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=indices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis,
                              useMaximalCoordinates=True)
pb.createConstraint(parentBodyUniqueId=ringID,parentLinkIndex=-1,childBodyUniqueId=ringID,childLinkIndex=n0-1,
                    jointType=pb.JOINT_FIXED,jointAxis=[0,1,0],
                    parentFramePosition=[0,0,0],
                    childFramePosition=[0,0,0],
                    parentFrameOrientation=[0,0,0,1],childFrameOrientation=[0,0,0,1])
restitutionAndFriction=[0.86,0.5,0.0005,0.002]
for joint in range(n0+1):
    pb.changeDynamics(ringID,joint,maxJointVelocity=0,jointDamping=1000,contactStiffness=5000,contactDamping=1,restitution=restitutionAndFriction[0],
                      lateralFriction=restitutionAndFriction[1],spinningFriction=restitutionAndFriction[2],rollingFriction=restitutionAndFriction[3])

velocity_0=[random.uniform(-1,1)*n0*5,random.uniform(-1,1)*n0*5,random.uniform(-1,1)*n0*5]
pb.resetBaseVelocity(ringID,linearVelocity=velocity_0)

pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,1)
for i in range(1000):
    velocity=pb.getBaseVelocity(ringID)[0]
    angularVelocity=EulerKineMaticaleQuationsTransform(ringID)
    position, _ = pb.getBasePositionAndOrientation(ringID)
    df.loc[i]["radiusOfRing_Ring"]=radiusOfRing_Ring
    df.loc[i]["radiusOfRing_section"]=radiusOfRing_section
    df.loc[i]["radiusOfRing_bar"]=radiusOfRing_bar
    df.loc[i]["mass"]=mass
    df.loc[i]["restitution"]=restitutionAndFriction[0]
    df.loc[i]["lateralFriction"]=restitutionAndFriction[1]
    df.loc[i]["spinningFriction"]=restitutionAndFriction[2]
    df.loc[i]["rollingFriction"]=restitutionAndFriction[3]
    df.loc[i]["position"]=np.round(position,2)
    df.loc[i]["velocity"]=np.round(velocity,2)
    df.loc[i]["angularVelocity"]=np.round(angularVelocity,2)
    df.loc[i]["time"]=i*nuitTime

    pb.stepSimulation()
    time.sleep(1/240)

    pb.resetDebugVisualizerCamera(cameraDistance=4,cameraYaw=130,cameraPitch=-40,cameraTargetPosition=[0,0,position[2]])
    # for joint in range(pb.getNumJoints(ringID)):
    #     pb.setJointMotorControlMultiDof(ringID,joint,pb.POSITION_CONTROL,targetPosition=[2*(radiusOfRing_section+radiusOfRing_Ring)*math.sin(math.pi/20),0,0],force=[100,100,100])
pb.disconnect()
print(df)
currenttime = datetime.datetime.now().replace(microsecond=0)
currenttime = currenttime.strftime("%Y-%m-%d,%H-%M-%S" + ".csv")
print("Writing data to file...\nName of file is :"+ "result " + currenttime +"\n")
df.to_csv("result_of ring on the bar " + currenttime)
