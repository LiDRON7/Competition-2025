class Aruco():
    def __init__(self, id):
        self.id = id
        self.distance_from_frame_center = None
        self.wetStatus = False
        self.timestamp = 0.00
        self.area = None
        self.X_coordinate = None
        self.Y_coordinate = None
        self.x_dist = None
        self.y_dist = None
        self.z_dist = None
        # self.nearest = False
       
    # in python, getters and setters are not necessary
           
    # def setDistance(self, distance):
    #     self.distance = distance
    
    # def setWetStatus(self, wetStatus):
    #     self.wetStatus = wetStatus

    # def setTimestamp(self, timestamp):
    #     self.timestamp = timestamp

    # def getDistance(self):
    #     return self.distance
    
    # def getWetStatus(self):
    #     return self.wetStatus
    
    def __str__(self):
        return f"Aruco ID: {self.id}, distance: {self.distance_from_frame_center}, area: {self.area}, wet? {self.wetStatus}"
