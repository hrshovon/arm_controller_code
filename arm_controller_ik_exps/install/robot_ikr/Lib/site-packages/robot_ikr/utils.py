class point:
    def __init__(self,x,y):
        self.x=x
        self.y=y

class rectangle:
    def __init__(self,p1,p2):
        assert isinstance(p1,point)
        assert isinstance(p2,point)
        self.p1=p1
        self.p2=p2    
    def get_center(self):
        center_point = point((self.p1.x+self.p2.x)//2,
                              (self.p1.y+self.p2.y)//2)
        return center_point