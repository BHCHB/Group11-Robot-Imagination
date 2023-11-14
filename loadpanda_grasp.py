import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import panda_sim_grasp as panda_sim


fps=300.
timeStep = 1./fps

def run():
	p.connect(p.GUI)

	# 设置仿真环境
	p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)	# 使Y轴向上, 默认Z轴向上
	# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
	p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
	p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw = 3, cameraPitch = -30, cameraTargetPosition = [0.35, -0.13, 0])

	p.setAdditionalSearchPath(pd.getDataPath())

	p.setTimeStep(timeStep)
	p.setGravity(0,-9.8,0)
	# 不展示GUI的套件
	p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


	# 初始化panda机器人
	panda = panda_sim.PandaSimAuto(p, [0,0,0])
	panda.control_dt = timeStep

	#载入地面模型，useMaximalCoordinates加大坐标刻度可以加快加载
	#p.loadURDF("plane.urdf", useMaximalCoordinates=True)
	#logId = panda.bullet_client.startStateLogging(panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
	#panda.bullet_client.submitProfileTiming("start")

	graspWidthId = p.addUserDebugParameter("graspWidth", 0, 0.06, 0.05)

	for i in range (10000):
		# panda.bullet_client.submitProfileTiming("full_step")
		graspWidth = float(p.readUserDebugParameter(graspWidthId))

		panda.step(graspWidth)

		p.stepSimulation()

		time.sleep(timeStep)

		# panda.bullet_client.submitProfileTiming()
	# panda.bullet_client.submitProfileTiming()
	# panda.bullet_client.stopStateLogging(logId)



if __name__ == "__main__":
	run()
