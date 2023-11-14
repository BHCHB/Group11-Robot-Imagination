import pybullet as p
import pybullet_data
import time

# 连接引擎
_ = p.connect(p.GUI)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)

# 载入地面模型，useMaximalCoordinates加大坐标刻度可以加快加载
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)

# 创建过程中不渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# 禁用 tinyrenderer
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

# 创建视觉模型和碰撞箱模型时共用的两个参数
#膨胀体与物体的偏移，此处表示碰撞体y上下移了0.02
shift = [0, 0, 0]
#缩放比例
scale = [0.1, 0.1, 0.1]
###########################################################
#workpiece
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="workpiece.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, 0.4, 0],
    visualFramePosition=shift,
    meshScale=scale
)

#shapetype可选GEOM_SPHERE, GEOM_BOX, GEOM_CAPSULE, GEOM_CYLINDER,
# GEOM_PLANE, GEOM_MESH
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="workpiece.obj",
    collisionFramePosition=[0, 0, 0],#表示实际碰撞体y轴上移动了2cm
    meshScale=scale
)

#for i in range(3):
# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
workpiece_test = p.createMultiBody(
        baseMass=50,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 5],
        useMaximalCoordinates=True
)

####################################################################
scale = [0.1, 0.1, 0.1]
# pen
visual_shape_pen = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="pen.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.5, 0.5, 0],
    visualFramePosition=shift,
    meshScale=scale
)


collision_shape_pen = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="pen.obj",
    collisionFramePosition=[0, 0, 0],#表示实际碰撞体y轴上移动了2cm
    meshScale=scale
)

# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
pen_test = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape_pen,
        baseVisualShapeIndex=visual_shape_pen,
        basePosition=[0, 0, 5],
        baseOrientation=[0,0,90,1],
        useMaximalCoordinates=True
)
#cid = p.createConstraint(pen_test, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])


# 创建结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


for i in range(10000):

    P_min, P_max = p.getAABB(workpiece_test)

    # getOverlapping的返回值为n个二元tuple，每个tuple的第一个值为与查询的对象发生碰撞的模型对象的ID，
    # tuple的第二个值为为那个模型对象的link的ID,如果不存在link则为-1
    id_tuple = p.getOverlappingObjects(P_min, P_max)

    # 机器人自己与自己重叠，所有getOverlappingObjects无论如何都会返回一个值，所以必须加if len(id_tuple) > 1:

    if len(id_tuple) > 2:
        ID = 2
        print(f"hit happen! hit object is {p.getBodyInfo(ID)}")
        cubePos, cubeOrn = p.getBasePositionAndOrientation(collision_shape_pen)
        print("-" * 20)
        print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
        print("-" * 20)

    # if len(id_tuple) > 2:
    #     for ID, _ in id_tuple:
    #         if ID == workpiece_test or ID == 1:
    #             continue
    #         else:
    #             print(f"hit happen! hit object is {p.getBodyInfo(ID)}")
    #             cubePos, cubeOrn = p.getBasePositionAndOrientation(collision_shape_pen)
    #             print("-" * 20)
    #             print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
    #             print("-" * 20)



    time.sleep(1./240.)


#getContactPoints(pen_test, workpiece_test)

p.disconnect()