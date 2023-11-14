import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.5, 0.458, 0.31, -1.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions

class PandaSim(object):
  def __init__(self, bullet_client, offset):

    self.bullet_client = bullet_client

    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)

    self.offset = np.array(offset)
    
    print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.legos=[]

    #tray / traybox
    self.bullet_client.loadURDF("plane.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)

    #self.legos.append(self.bullet_client.loadURDF("scew/urdf/scew.urdf",np.array([0.1, 0.3, -0.5])+self.offset, flags=flags))
    #self.bullet_client.changeVisualShape(self.legos[0],-1,rgbaColor=[1,0,0,1])

    # 将 lego 模型旋转到与地面平行的姿态（例如，绕X轴旋转90度）
    lego_orientation = self.bullet_client.getQuaternionFromEuler([np.pi*1.5, np.pi/2, 0])

   # self.bullet_client.loadURDF("imagination.urdf", [2, 2, 2], lego_orientation, flags=flags)

    # 创建 screw 模型
    self.legos.append(
      self.bullet_client.loadURDF("modeling/scew/urdf/scew.urdf", np.array([-0.15, 0.15, -0.75]) + self.offset, lego_orientation,
                                  flags=flags))
    self.legos.append(
      self.bullet_client.loadURDF("cube.urdf", np.array([0.29, 0.05, -0.57]) + self.offset, lego_orientation,
                                  flags=flags,globalScaling=0.005,useFixedBase=True,))

    orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("modeling/franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)

    self.bullet_client.loadURDF("modeling/imagination/urdf/imagination.urdf", np.array([0.5, 0, -0.7]), [-0.707107, 0.0,  0, 0.707107],useFixedBase=True, flags=flags)
    index = 0
    self.state = 0
    self.control_dt = 1./240.
    self.finger_target = 0
    self.gripper_height = 0.2
    #create a constraint to keep the fingers centered
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-0.5, erp=0.1, maxForce=25)
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1

      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.
  def reset(self):
    pass
  def update_state(self):
    keys = self.bullet_client.getKeyboardEvents()
    if len(keys)>0:
      for k,v in keys.items():
        if v&self.bullet_client.KEY_WAS_TRIGGERED:
          if (k==ord('0')):
            self.state = 0
          if (k==ord('1')):
            self.state = 1
          if (k==ord('2')):
            self.state = 2
          if (k==ord('3')): # 物体上方 预抓取位置
            self.state = 3
          if (k==ord('4')): # 下移
            self.state = 4
          if (k==ord('5')): # 机械手张开
                self.state = 5
          if (k==ord('6')): # 机械手闭合
                self.state = 6
          if (k==ord('7')): # 回到中心点
                self.state = 7
          if (k == ord('8')):
                  self.state = 8
          if (k == ord('9')):
            self.state = 9
        if v&self.bullet_client.KEY_WAS_RELEASED:
            self.state = 0
  def step(self, graspWidth):
    # 设置抓取器张开宽度
    if self.state==6:
      self.finger_target = 0.002
    if self.state==5:
      self.finger_target = 0.04 
    
    # 测试用
    # self.finger_target = graspWidth
    # # print('self.finger_target = ', self.finger_target)
    # for i in [9,10]:
    #   self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 10)

    self.update_state()
    # print("self.state=",self.state)
    #print("self.finger_target=",self.finger_target)

    alpha = 0.9 #0.99
    if self.state==1 or self.state==2 or self.state==3 or self.state==4 or self.state==7 or self.state==8 or self.state==9:
      #gripper_height = 0.034
      self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.03
      # print('self.gripper_height = ', self.gripper_height)

      if self.state == 2 or self.state == 3 or self.state == 7 or self.state==8:
        self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.2
        # print('self.gripper_height = ', self.gripper_height)

      t = self.t
      self.t += self.control_dt
      pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+self.gripper_height, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)] # 圆形位置

      if self.state == 3 or self.state== 4:
        # 获取screw的位置和方向
        pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[0])    #红色积木 self.legos[0]
        pos = [pos[0], self.gripper_height+0.05, pos[2]] # 机械手位置x,y,z 之前将z和y轴互换了，y为高度
        self.prev_pos = pos

      if self.state == 7:
         pos = self.prev_pos
         diffX = pos[0] - self.offset[0]
         diffZ = pos[2] - (self.offset[2]-0.6)
         self.prev_pos = [self.prev_pos[0] - diffX*0.1, self.prev_pos[1], self.prev_pos[2]-diffZ*0.1]

      if self.state == 8 :
        # 获取积木的位置和方向
        pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[1])  # 红色积木 self.legos[0]
        pos = [pos[0], self.gripper_height + 0.3, pos[2]]  # 机械手位置x,y,z 之前将z和y轴互换了，y为高度
        self.prev_pos = pos

      if self.state == 9:
        # 获取积木的位置和方向
        pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[1])  # 红色积木 self.legos[0]
        pos = [pos[0], self.gripper_height + 0.15, pos[2]]  # 机械手位置x,y,z 之前将z和y轴互换了，y为高度
        self.prev_pos = pos

      orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])   # 机械手方向
      # 根据目标位置计算关节位置
      jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20)

      for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)

    #target for fingers
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 50)

class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, offset):
    PandaSim.__init__(self, bullet_client, offset)
    self.state_t = 0
    self.cur_state = 0
    self.states=[0,5, 3, 5, 4, 6,3, 8, 9,5]#操作顺序
    self.state_durations=[0.5, 0.25,0.25, 0.25, 0.25,0.25,0.5, 0.9,0.9,0.25]#每个动作的时间

  
  def update_state(self):
    self.state_t += self.control_dt
    if self.state_t > self.state_durations[self.cur_state]:
      self.cur_state += 1
      if self.cur_state >= len(self.states):
        self.cur_state = 0
      self.state_t = 0
      self.state=self.states[self.cur_state]
      #print("self.state=",self.state)
