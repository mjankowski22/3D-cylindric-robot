from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Point3,Point2
from panda3d.core import Vec3
from panda3d.core import CollisionTraverser, CollisionNode
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionRay,DirectionalLight
import math
import simplepbr




class RobotSimulation(ShowBase):
    def __init__(self):
        
        ShowBase.__init__(self)
        simplepbr.init()

        # Load the robot model``
        self.robot = self.loader.loadModel("robot2wersja.gltf")
        self.robot.reparentTo(self.render)
        self.robot.setPos(0, 0, 1)
        
        self.cone = self.robot.get_children()[0]
        self.cylinder = self.robot.get_children()[1]
        self.up_cylinder = self.robot.get_children()[2]
        self.first_arm = self.robot.get_children()[3]
        self.second_arm = self.robot.get_children()[4]
        self.collector_base = self.robot.get_children()[5]
        self.collector_right = self.robot.get_children()[6]
        self.collector_left = self.robot.get_children()[7]


        self.to_move_up_down = [self.collector_base,self.collector_right,self.collector_left,self.first_arm,self.second_arm,self.up_cylinder]
        self.to_rotate = [self.collector_base,self.collector_right,self.collector_left,self.first_arm,self.second_arm]
        self.to_go_forward = [self.collector_base,self.second_arm]
        self.to_expand = [self.collector_left,self.collector_right]
        


        # self.floor = self.loader.loadModel("Floor.gltf")
        # self.floor.reparentTo(self.render)
        # self.floor.setPos(0,0,0)
        
        self.disable_mouse()
        # Set initial position and orientation of the robot
        self.camera_r = 35
        self.camera_t = 0.55
        self.camera_fi = 0

        self.speed_up =0.1
        self.speed_rotate = 0.02
        self.speed_expand =0.01
        self.speed_forward = 0.1
        
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))  # Initial camera position
        self.camera.look_at(Point3(0, 0, 0))  # Look at the origin
        
        self.accept('arrow_up-repeat', self.rotate_up)
        self.accept('arrow_down-repeat', self.rotate_down)
        self.accept(',',self.zoom_in)
        self.accept('.',self.zoom_out)
        self.accept('arrow_left-repeat', self.rotate_right)
        self.accept('arrow_right-repeat', self.rotate_left)
        self.accept('w-repeat',self.move_up)
        self.accept('s-repeat',self.move_down)
        self.accept('a-repeat',self.robot_rotate_left)
        self.accept('d-repeat',self.robot_rotate_right)
        self.accept('q-repeat',self.go_forward)
        self.accept('e-repeat',self.go_backward)
        self.accept('r-repeat',self.reduce)
        self.accept('t-repeat',self.expand)

        self.accept(',-repeat',self.zoom_in)
        self.accept('.-repeat',self.zoom_out)
        self.accept('arrow_up', self.rotate_up)
        self.accept('arrow_down', self.rotate_down)
        self.accept('arrow_left', self.rotate_right)
        self.accept('arrow_right', self.rotate_left)
        self.accept('w',self.move_up)
        self.accept('s',self.move_down)
        self.accept('a',self.robot_rotate_left)
        self.accept('d',self.robot_rotate_right)
        self.accept('q',self.go_forward)
        self.accept('e',self.go_backward)
        self.accept('r',self.reduce)
        self.accept('t',self.expand)
        

        self.rotate_speed = 0.2 # Adjust the speed of rotation
        
        
        self.taskMgr.add(self.update_camera_task, 'update_camera_task')
    
    def zoom_in(self):
        if self.camera_r>22:
            self.camera_r=self.camera_r*0.9
            self.camera.set_pos(self.camera.get_pos() * 0.9)  # Zoom in by scaling the position
    
    def zoom_out(self):
        if self.camera_r<45:
            self.camera_r=self.camera_r*1.1
            self.camera.set_pos(self.camera.get_pos() * 1.1)  # Zoom out by scaling the position
    
    def rotate_left(self):
        self.camera_fi += self.rotate_speed  # Increment the rotation angle
        self.rotate_camera()
        
    def rotate_right(self):
        self.camera_fi -= self.rotate_speed  # Decrement the rotation angle
        self.rotate_camera()
    
    def rotate_up(self):
        if self.camera_t<1.54:
            self.camera_t+=self.rotate_speed
            self.rotate_camera()

    def rotate_down(self):
        if self.camera_t >0.3:
            self.camera_t-=self.rotate_speed
            self.rotate_camera()

    def rotate_camera(self):
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))
        self.camera.look_at(Point3(0, 0, 0))  # Keep looking at the origin

    def move_up(self):
        for element in self.to_move_up_down:
            pos = list(element.getPos())
            if pos[2]<=7.1:
                element.set_pos(pos[0],pos[1],pos[2]+self.speed_up)

    def move_down(self):
        for element in self.to_move_up_down:
            pos = list(element.getPos())
            if pos[2]>=1.9:
                element.set_pos(pos[0],pos[1],pos[2]-self.speed_up)

    def robot_rotate_left(self):
        for element in self.to_rotate:
            pos = list(element.getPos())
            actual_fi = math.atan2(pos[1],pos[0])
            actual_r = math.sqrt(pos[0]**2+pos[1]**2)
            fi = actual_fi + self.speed_rotate
            element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
            element.setH(fi/(2*math.pi)*360)

    def robot_rotate_right(self):
        for element in self.to_rotate:
            pos = list(element.getPos())
            actual_fi = math.atan2(pos[1],pos[0])
            actual_r = math.sqrt(pos[0]**2+pos[1]**2)
            fi = actual_fi - self.speed_rotate
            element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
            element.setH(fi/(2*math.pi)*360)

    def go_forward(self):
        
        if math.sqrt(self.second_arm.getPos()[0]**2+ self.second_arm.getPos()[1]**2) <=5.8:
            appends_left = [self.collector_left.getPos()[0]-self.collector_base.getPos()[0],self.collector_left.getPos()[1]-self.collector_base.getPos()[1]]
            appends_right = [self.collector_right.getPos()[0]-self.collector_base.getPos()[0],self.collector_right.getPos()[1]-self.collector_base.getPos()[1]]

            for element in self.to_go_forward:
                pos = list(element.getPos())
                actual_r = math.sqrt(pos[0]**2+pos[1]**2)
                fi = math.atan2(pos[1],pos[0])
                actual_r +=self.speed_forward
                element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
            self.collector_left.set_pos(self.collector_base.getPos()[0]+appends_left[0],self.collector_base.getPos()[1]+appends_left[1],self.collector_left.getPos()[2])
            self.collector_right.set_pos(self.collector_base.getPos()[0]+appends_right[0],self.collector_base.getPos()[1]+appends_right[1],self.collector_right.getPos()[2])

    def go_backward(self):
        appends_left = [self.collector_left.getPos()[0]-self.collector_base.getPos()[0],self.collector_left.getPos()[1]-self.collector_base.getPos()[1]]
        appends_right = [self.collector_right.getPos()[0]-self.collector_base.getPos()[0],self.collector_right.getPos()[1]-self.collector_base.getPos()[1]]
        if math.sqrt(self.second_arm.getPos()[0]**2+ self.second_arm.getPos()[1]**2) >=2.5:
            for element in self.to_go_forward:
                pos = list(element.getPos())
                actual_r = math.sqrt(pos[0]**2+pos[1]**2)
                fi = math.atan2(pos[1],pos[0])
                actual_r -=self.speed_forward
                element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
            self.collector_left.set_pos(self.collector_base.getPos()[0]+appends_left[0],self.collector_base.getPos()[1]+appends_left[1],self.collector_left.getPos()[2])
            self.collector_right.set_pos(self.collector_base.getPos()[0]+appends_right[0],self.collector_base.getPos()[1]+appends_right[1],self.collector_right.getPos()[2])

    def expand(self):
        pos_c_l = list(self.collector_left.getPos())
        pos_c_r = list(self.collector_right.getPos())
        r_c_l = math.sqrt(pos_c_l[0]**2+pos_c_l[1]**2)
        fi_c_l = math.atan2(pos_c_l[1],pos_c_l[0])
        r_c_r = math.sqrt(pos_c_r[0]**2+pos_c_r[1]**2)
        fi_c_r = math.atan2(pos_c_r[1],pos_c_r[0])
        
        if math.sqrt((pos_c_l[0]-pos_c_r[0])**2+(pos_c_l[1]-pos_c_r[1])**2)<1.35:
            fi_c_l+=self.speed_expand
            fi_c_r-=self.speed_expand
            self.collector_left.set_pos(r_c_l*math.cos(fi_c_l), r_c_l*math.sin(fi_c_l), pos_c_l[2])
            self.collector_right.set_pos(r_c_r*math.cos(fi_c_r), r_c_r*math.sin(fi_c_r), pos_c_r[2])

            
    def reduce(self):
        pos_c_l = list(self.collector_left.getPos())
        pos_c_r = list(self.collector_right.getPos())
        r_c_l = math.sqrt(pos_c_l[0]**2+pos_c_l[1]**2)
        fi_c_l = math.atan2(pos_c_l[1],pos_c_l[0])
        r_c_r = math.sqrt(pos_c_r[0]**2+pos_c_r[1]**2)
        fi_c_r = math.atan2(pos_c_r[1],pos_c_r[0])
        if math.sqrt((pos_c_l[0]-pos_c_r[0])**2+(pos_c_l[1]-pos_c_r[1])**2)>0.5:
            fi_c_l-=self.speed_expand
            fi_c_r+=self.speed_expand
            self.collector_left.set_pos(r_c_l*math.cos(fi_c_l), r_c_l*math.sin(fi_c_l), pos_c_l[2])
            self.collector_right.set_pos(r_c_r*math.cos(fi_c_r), r_c_r*math.sin(fi_c_r), pos_c_r[2])


    def update_camera_task(self, task):
        return task.cont

simulation = RobotSimulation()
simulation.run()
