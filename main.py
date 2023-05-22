from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Point3,Point2
from panda3d.core import Vec3
from panda3d.core import CollisionTraverser, CollisionNode
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionRay,DirectionalLight
import math
from camera_controller import CameraController
import simplepbr



class RobotSimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        simplepbr.init()

        # Load the robot model
        self.robot = self.loader.loadModel("Robot.gltf")
        self.robot.reparentTo(self.render)
        self.robot.setPos(0, 0, 1)

        self.floor = self.loader.loadModel("Floor.gltf")
        self.floor.reparentTo(self.render)
        self.floor.setPos(0,0,0)
        
        self.disable_mouse()
        # Set initial position and orientation of the robot
        self.camera_r = 20
        self.camera_t = 0.5
        self.camera_fi = 0
        
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))  # Initial camera position
        self.camera.look_at(Point3(0, 0, 0))  # Look at the origin
        
        self.accept('arrow_up-repeat', self.zoom_in)
        self.accept('arrow_down-repeat', self.zoom_out)
        self.accept('arrow_left-repeat', self.rotate_left)
        self.accept('arrow_right-repeat', self.rotate_right)
        

        self.rotate_speed = 0.2 # Adjust the speed of rotation
        
        
        self.taskMgr.add(self.update_camera_task, 'update_camera_task')
    
    def zoom_in(self):
        self.camera_r=self.camera_r*0.9
        self.camera.set_pos(self.camera.get_pos() * 0.9)  # Zoom in by scaling the position
    
    def zoom_out(self):
        self.camera_r=self.camera_r*1.1
        self.camera.set_pos(self.camera.get_pos() * 1.1)  # Zoom out by scaling the position
    
    def rotate_left(self):
        self.camera_fi += self.rotate_speed  # Increment the rotation angle
        self.rotate_camera()
        
    def rotate_right(self):
        self.camera_fi -= self.rotate_speed  # Decrement the rotation angle
        self.rotate_camera()
    
    def rotate_camera(self):
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))
        self.camera.look_at(Point3(0, 0, 0))  # Keep looking at the origin
    
    def update_camera_task(self, task):
        return task.cont

simulation = RobotSimulation()
simulation.run()
