from robot_control_class import RobotControl
import numpy as np

class Project:

    def __init__(self,speed,time):
        self.rc=RobotControl()
        self.motion=None
        self.speed=speed
        self.time=time
        self.d=None
    def dist(self):
        return(self.rc.get_laser(360))
    def mov(self):
        distance=self.dist()
        while True:
            while (distance>1):
                self.rc.move_straight()
                distance=self.dist()
                print("Current distance from wall : ", distance)

            self.rc.stop_robot()
            self.motion=self.where_turn()
            self.rc.turn(self.motion,self.speed,self.time)
            print("turning ", self.motion)
            distance=self.dist()
    def where_turn(self):
        self.d=self.rc.get_laser_full()
        l1=[]
        l2=[]
        cnt=len(self.d)
        i=0
        j=cnt/2
        while(i<cnt/2):
            l1.append(self.d[i])
            i=i+1
        while(j<cnt):
            l2.append(self.d[j])
            j=j+1
        v1,v2=self.mean(l1,l2)
        print("sum of right= ",v1)
        print("sum of left= ",v2)

        if(v1>v2):
            self.d=None
            return "clockwise"
        else:
            self.d=None
            return "counter-clockwise"

    def mean(self,x,y):
        x=np.asarray(x)
        y=np.asarray(y)
        m1=np.sum(x)
        m2=np.sum(y)
        return [m1,m2]

robot=Project(speed=0.29,time=5)
robot.mov()




'''count=len(a)
i=0
j=count/2
while(i<count/2):
    l1.append(a[i])
    i=i+1
while(j<count):
    l2.append(a[j])
    j=j+1
l1=np.asarray(l1)
l2=np.asarray(l2)
m1=np.mean(l1)
m2=np.mean(l2)'''