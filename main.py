from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Point3,Point2
from panda3d.core import Vec3
from panda3d.core import CollisionTraverser, CollisionNode
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionRay,DirectionalLight
import math
import simplepbr
from direct.gui.DirectGui import *
from time import sleep

#ll

class RobotSimulation(ShowBase):
    def __init__(self):
        
        ShowBase.__init__(self)
        simplepbr.init()

        # Load the robot model``
        self.robot = self.loader.loadModel("Robot4.gltf")
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
        self.primitive = self.robot.get_children()[9]

        self.to_move_up_down = [self.collector_base,self.collector_right,self.collector_left,self.first_arm,self.second_arm,self.up_cylinder]
        self.to_rotate = [self.collector_base,self.collector_right,self.collector_left,self.first_arm,self.second_arm]
        self.to_go_forward = [self.collector_base,self.second_arm]
        self.to_expand = [self.collector_left,self.collector_right]
        
        self.record = []
        self.recording = False

        self.buttonRecord = DirectButton(text="Rozpocznij nagrywanie",
            scale=0.1,
            frameColor=(0.4, 0.6, 0.8, 1),  # RGBA values for the frame color
            frameSize=(-2.3,2.3, -0.5, 0.5),  # Width and height of the button
            pos=(-0.9, 0, 0.9),  # Position of the button (upper-left corner)
            text_scale=0.4,  # Scaling of the button text
            text_pos=(0, -0.05),
            command=self.start_recording  # Position of the text on the button
        )
        self.buttonRecord.setPos(1.1, 0, 0.9)

        self.buttonPlay = DirectButton(text="Odtworz",
            scale=0.1,
            frameColor=(0.4, 0.6, 0.8, 1),  # RGBA values for the frame color
            frameSize=(-1, 1, -0.5, 0.5),  # Width and height of the button
            pos=(-0.9, 0, 0.9),  # Position of the button (upper-left corner)
            text_scale=0.4,  # Scaling of the button text
            text_pos=(0, -0.05),
            command=self.play 
        )
        self.buttonPlay.setPos(1.23, 0, -0.9)

        self.buttonReset = DirectButton(text="Reset kamery",
            scale=0.1,
            frameColor=(0.4, 0.6, 0.8, 1),  # RGBA values for the frame color
            frameSize=(-1.4, 1.4, -0.5, 0.5),  # Width and height of the button
            pos=(-0.9, 0, 0.9),  # Position of the button (upper-left corner)
            text_scale=0.4,  # Scaling of the button text
            text_pos=(0, -0.05), 
            command=self.camera_reset # Position of the text on the button
        )
        self.buttonReset.setPos(-1.2, 0, 0.9)

        self.buttonMove = DirectButton(text="Ruch",
            scale=0.1,
            frameColor=(0.4, 0.6, 0.8, 1),  # RGBA values for the frame color
            frameSize=(-1, 1, -0.5, 0.5),  # Width and height of the button
            pos=(-1.21, 0, -0.7),  # Position of the button (upper-left corner)
            text_scale=0.4,  # Scaling of the button text
            text_pos=(0, -0.05),
            command = self.move # Position of the text on the button
        )
        self.buttonMove.setPos(-1.2, 0, -0.7)

        self.x_input = DirectEntry(
            scale=0.06,
            initialText="x",
            width=5,
            pos=(-1.3, 0, -0.9)
        )
        self.y_input = DirectEntry(
            scale=0.06,
            initialText="y",
            width=5,
            pos=(-0.9, 0, -0.9)
        )
        self.z_input = DirectEntry(
            scale=0.06,
            initialText="z",
            width=5,
            pos=(-0.5, 0, -0.9)
        )
        self.inputs = [self.x_input,self.y_input,self.z_input]
        
        
        
        self.disable_mouse()
        # Set initial position and orientation of the robot
        self.camera_r = 35
        self.camera_t = 0.55
        self.camera_fi = -0.15

        self.speed_up =0.1
        self.speed_rotate = 0.02
        self.speed_expand =0.01
        self.speed_forward = 0.1
        self.is_catched = False
        
        print(self.primitive.getPos())

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

        self.accept('mouse1',self.checkClick)
        

        self.rotate_speed = 0.2 # Adjust the speed of rotation
        
    
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

    def camera_reset(self):
        self.camera_r = 35
        self.camera_t = 0.55
        self.camera_fi = -0.15
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))  # Initial camera position
        self.camera.look_at(Point3(0, 0, 0))  # Look at the origin

    def rotate_camera(self):
        self.camera.set_pos(self.camera_r*math.cos(self.camera_t)*math.cos(self.camera_fi), self.camera_r*math.cos(self.camera_t)*math.sin(self.camera_fi), self.camera_r*math.sin(self.camera_t))
        self.camera.look_at(Point3(0, 0, 0))  # Keep looking at the origin

    def move_up(self):
        if self.recording:
            self.record.append(self.move_up)
        self.move_primitive()        
        for element in self.to_move_up_down:
            pos = list(element.getPos())
            if pos[2]<=7.1 :
                if element==self.primitive:
                    element.set_pos(pos[0],pos[1],pos[2]+self.speed_up)
                else:
                    element.set_pos(pos[0],pos[1],pos[2]+self.speed_up)  
                 

    def move_down(self):
        if self.recording:
            self.record.append(self.move_down)
        self.move_primitive()
        for element in self.to_move_up_down:
            pos = list(element.getPos())
            if pos[2]>=1.9 :
                if element==self.primitive:
                    element.set_pos(pos[0],pos[1],pos[2]-self.speed_up)
                else:
                    element.set_pos(pos[0],pos[1],pos[2]-self.speed_up)  
                

    def robot_rotate_left(self):
        if self.recording:
            self.record.append(self.robot_rotate_left)
        self.move_primitive()
        if not self.check_collision_with_barell_left() and not self.check_collision_with_table_left() and(( not self.check_collision_with_primitive_left() and not self.check_collision_with_primitive_middle()) or (self.check_collision_with_primitive_left() and self.check_collision_with_primitive_right())) :
            
            for element in self.to_rotate:
                pos = list(element.getPos())
                actual_fi = math.atan2(pos[1],pos[0])
                actual_r = math.sqrt(pos[0]**2+pos[1]**2)
                fi = actual_fi + self.speed_rotate
                element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
                element.setH(fi/(2*math.pi)*360)

    def robot_rotate_right(self):
        if self.recording:
            self.record.append(self.robot_rotate_right)
        self.move_primitive()
        if not self.check_collision_with_barell_right() and not self.check_collision_with_table_right() and(( not self.check_collision_with_primitive_right() and not self.check_collision_with_primitive_middle()) or (self.check_collision_with_primitive_left() and self.check_collision_with_primitive_right())) :
            
            for element in self.to_rotate:
                pos = list(element.getPos())
                actual_fi = math.atan2(pos[1],pos[0])
                actual_r = math.sqrt(pos[0]**2+pos[1]**2)
                fi = actual_fi - self.speed_rotate
                element.set_pos(actual_r*math.cos(fi), actual_r*math.sin(fi), pos[2])
                element.setH(fi/(2*math.pi)*360)

    def go_forward(self):
        if self.recording:
            self.record.append(self.go_forward)
        self.move_primitive()
        if  not self.check_collision_with_barell_left() and not self.check_collision_with_barell_right() and not self.check_collision_with_table_left() and not self.check_collision_with_table_right() and (( not self.check_collision_with_primitive_right() and not self.check_collision_with_primitive_left() and not self.check_collision_with_primitive_middle()) or (self.check_collision_with_primitive_left() and self.check_collision_with_primitive_right())):
            
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
        if self.recording:
            self.record.append(self.go_backward)
        self.move_primitive()
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
        if self.recording:
            self.record.append(self.expand)
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
        self.fall_primitive()

        

            
    def reduce(self):
        if self.recording:
            self.record.append(self.reduce)
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

    def check_collision_with_barell_left(self):

        left_x,left_y,left_z = self.collector_left.getPos()[0],self.collector_left.getPos()[1],self.collector_left.getPos()[2]
        left_r = math.sqrt(left_x**2+left_y**2)
        left_fi = math.atan2(left_y,left_x)
        left_r +=0.6
        left_x = left_r*math.cos(left_fi)
        left_y = left_r*math.sin(left_fi)
        if (left_x - 2)**2 + (left_y+8)**2 <= 2**2 and left_z<6.3:
            return True
        else:
            return False
        
    def check_collision_with_barell_right(self):

        right_x,right_y,right_z = self.collector_right.getPos()[0],self.collector_right.getPos()[1],self.collector_right.getPos()[2]
        right_r = math.sqrt(right_x**2+right_y**2)
        right_fi = math.atan2(right_y,right_x)
        right_r+=0.6
        right_x = right_r*math.cos(right_fi)
        right_y = right_r*math.sin(right_fi)
        if (right_x - 2)**2 + (right_y+8)**2 <= 2**2 and right_z<6.3:
            return True
        else:
            return False
        

    def check_collision_with_table_left(self):

        left_x,left_y,left_z = self.collector_left.getPos()[0],self.collector_left.getPos()[1],self.collector_left.getPos()[2]
        left_r = math.sqrt(left_x**2+left_y**2)
        left_fi = math.atan2(left_y,left_x)
        left_r +=0.6
        left_x = left_r*math.cos(left_fi)
        left_y = left_r*math.sin(left_fi)
        if (left_x - 2)**2 + (left_y-9)**2 <= 3**2 and left_z<4.4 and left_z>3.4:
            return True
        else:
            return False
        
    def check_collision_with_table_right(self):

        right_x,right_y,right_z = self.collector_right.getPos()[0],self.collector_right.getPos()[1],self.collector_right.getPos()[2]
        right_r = math.sqrt(right_x**2+right_y**2)
        right_fi = math.atan2(right_y,right_x)
        right_r+=0.6
        right_x = right_r*math.cos(right_fi)
        right_y = right_r*math.sin(right_fi)
        if (right_x -2)**2 + (right_y-9)**2 <= 3**2 and right_z<4.4 and right_z>3.4:
            return True
        else:
            return False
        
    def check_collision_with_primitive_left(self):
        left_x,left_y,left_z = self.collector_left.getPos()[0],self.collector_left.getPos()[1],self.collector_left.getPos()[2]
        left_r = math.sqrt(left_x**2+left_y**2)
        left_fi = math.atan2(left_y,left_x)
        r = -0.5
        actual_r =0 
        while r!=0.6:
            actual_r = left_r+r
            left_x = actual_r*math.cos(left_fi)
            left_y = actual_r*math.sin(left_fi)
            primitive_x,primitive_y,primitive_z = self.primitive.getPos()[0],self.primitive.getPos()[1],self.primitive.getPos()[2]
            if (left_x -primitive_x)**2 + (left_y- primitive_y)**2 + (left_z-0.2-primitive_z)**2 <= 0.5**2 :
                return True
            r+=0.1
        return False

                
        
    def check_collision_with_primitive_right(self):
        right_x,right_y,right_z = self.collector_right.getPos()[0],self.collector_right.getPos()[1],self.collector_right.getPos()[2]
        right_r = math.sqrt(right_x**2+right_y**2)
        right_fi = math.atan2(right_y,right_x)
        r=-0.5
        actual_r=0
        while r!=0.6:
            actual_r = right_r +r
            right_x = actual_r*math.cos(right_fi)
            right_y = actual_r*math.sin(right_fi)
            primitive_x,primitive_y,primitive_z = self.primitive.getPos()[0],self.primitive.getPos()[1],self.primitive.getPos()[2]
            if (right_x -primitive_x)**2 +(right_y- primitive_y)**2 +(right_z-0.2-primitive_z)**2 <= 0.5**2 :
                return True
            r+=0.1
        return False
    
    def check_collision_with_primitive_middle(self):
        middle_x,middle_y,middle_z = self.collector_base.getPos()[0],self.collector_base.getPos()[1],self.collector_base.getPos()[2]
        middle_r = math.sqrt(middle_x**2+middle_y**2)+0.08
        middle_fi = math.atan2(middle_y,middle_x)
        fi=-math.pi/36*2
        actual_fi = 0
        while fi!=math.pi/36*2:
            actual_fi = middle_fi +fi
            middle_x = middle_r*math.cos(actual_fi)
            middle_y = middle_r*math.sin(actual_fi)
            primitive_x,primitive_y,primitive_z = self.primitive.getPos()[0],self.primitive.getPos()[1],self.primitive.getPos()[2]
            if (middle_x -primitive_x)**2 +(middle_y- primitive_y)**2 +(middle_z-0.2-primitive_z)**2 <= 0.5**2 :
                return True
            fi+=math.pi/36
        return False
    
    def checkClick(self):
        # Check if the mouse was clicked outside of the text inputs
        mouse_pos = self.mouseWatcherNode.getMouse()
        mouse_x = mouse_pos[0]
        mouse_y = mouse_pos[1]
        for input_field in self.inputs:
            if (
                mouse_x >= input_field.getPos().getX() and
                mouse_x <= input_field.getPos().getX() + input_field.getWidth() and
                mouse_y >= input_field.getPos().getY() and
                mouse_y <= input_field.getPos().getY() + input_field.getHeight()
            ):
                continue
            else:
                input_field.setFocus()

    def start_recording(self):
        if not self.recording:
            self.prev_pos = []
            self.prev_hprs = []
            for element in self.robot.get_children():
                self.prev_pos.append(element.getPos())
                self.prev_hprs.append(element.getHpr())
            self.buttonRecord.setText("Zatrzymaj nagrywanie")
            self.recording = True
            self.record = []
        else:
            for i in range(len(self.prev_pos)):
                self.robot.get_children()[i].setPos(self.prev_pos[i])
                self.robot.get_children()[i].setHpr(self.prev_hprs[i])
            self.buttonRecord.setText("Rozpocznij nagrywanie")
            self.recording = False

    def play(self):
        if not self.recording and self.record != []:
        
            for i in range(len(self.prev_pos)):
                self.robot.get_children()[i].setPos(self.prev_pos[i])
                self.robot.get_children()[i].setHpr(self.prev_hprs[i])
            self.i=0
            self.task_mgr.doMethodLater(0.07,self.play_task,"play_task")
                
    def play_task(self,task):
        try:
            self.record[self.i]()
            self.i +=1
        except:
            self.task_mgr.remove("play_task")
        return task.again
    
    def move(self):
        try:
            x = float(self.x_input.get())
            y = float(self.y_input.get())
            z = float(self.z_input.get())
            self.r = math.sqrt(x**2+y**2)
            self.fi = math.atan2(y,x) + math.pi
            self.z = z
            self.task_mgr.doMethodLater(0.07,self.move_task,"move_task")
            self.flag = False
        except:
            pass
    
    def move_task(self,task):
        
        x,y,z = self.collector_base.getPos()[0],self.collector_base.getPos()[1],self.collector_base.getPos()[2]
        r = math.sqrt(x**2+y**2)
        fi = math.atan2(y,x)+math.pi
        print(f"obecne: {z}")
        print(self.z)
        go_z =6
        if z-go_z>0.12 and not self.flag:
            self.move_down()
            return task.again
        elif z-go_z<-0.12 and not self.flag:
            self.move_up()
            return task.again
        else:
            self.flag=True
        if r+0.55-self.r>0.12:
            self.go_backward()
            return task.again
        elif r+0.55-self.r<-0.12:
            self.go_forward()
            return task.again
        if abs(fi-self.fi)>0.021:
            if fi-self.fi>math.pi:
                self.robot_rotate_right()
                return task.again
            elif fi-self.fi<math.pi:
                self.robot_rotate_left()
                return task.again
        if z-self.z>0.12:
            self.move_down()
            return task.again
        elif z-self.z<-0.12:
            self.move_up()
            return task.again
        
        # elif abs(fi-self.fi)<0.037:
        #     self.robot_rotate_right()
        self.task_mgr.remove('move_task')

    def check_catch_primitive(self):
        if self.check_collision_with_primitive_right() and self.check_collision_with_primitive_left():
            return True
        else:
            self.is_catched = False
            return False
        
    def set_to_move_primitive(self):
        self.to_move_up_down.append(self.primitive)
        self.to_rotate.append(self.primitive)
        self.to_go_forward.append(self.primitive)
        self.is_catched = True
        
    def stop_move_primitive(self):
        self.to_move_up_down.remove(self.primitive)
        self.to_rotate.remove(self.primitive)
        self.to_go_forward.remove(self.primitive)

        
    def move_primitive(self):
        if self.check_catch_primitive() and not self.is_catched:
            self.set_to_move_primitive()

    def check_collision_with_barell_primitive(self):

        prim_x,prim_y,prim_z = self.primitive.getPos()[0],self.primitive.getPos()[1],self.primitive.getPos()[2]
        prim_r = math.sqrt(prim_x**2+prim_y**2)
        prim_fi = math.atan2(prim_y,prim_x)
        prim_r +=0.6
        prim_x = prim_r*math.cos(prim_fi)
        prim_y = prim_r*math.sin(prim_fi)
        if (prim_x - 2)**2 + (prim_y+8)**2 <= 2**2 and prim_z<6.3:
            return True
        else:
            return False
    
    def prepare_fall(self,task):
        pos = list(self.primitive.getPos())
        prim_x,prim_y,prim_z = self.primitive.getPos()[0],self.primitive.getPos()[1],self.primitive.getPos()[2]
        prim_r = math.sqrt(prim_x**2+prim_y**2)
        prim_fi = math.atan2(prim_y,prim_x)
        prim_r +=0.6
        prim_x = prim_r*math.cos(prim_fi)
        prim_y = prim_r*math.sin(prim_fi)
        if (prim_x - 2)**2 + (prim_y+8)**2 <= 2**2 and prim_z<6.5:
            self.task_mgr.remove('prepare_fall')
        if (prim_x - 2)**2 + (prim_y-9)**2 <= 3**2 and prim_z<4.6:
            self.task_mgr.remove('prepare_fall')
        if pos[2] >= 0.4:
            self.primitive.set_pos(pos[0],pos[1],pos[2] - 0.08)
            return task.again
        self.task_mgr.remove('prepare_fall')
        

    

    def fall_primitive(self):
        if self.is_catched and not self.check_catch_primitive():
            self.stop_move_primitive()
            self.task_mgr.doMethodLater(0.0001,self.prepare_fall,"prepare_fall")
            
            
    
        



    




simulation = RobotSimulation()
simulation.run()
