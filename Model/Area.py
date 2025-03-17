class Area:
    def __init__(self, parking_spots = {}, borders = {}):
        self.parking_spots = parking_spots
        self.borders = borders
        
    def update_parking_spots(self, parking_spots):
        self.parking_spots = parking_spots
    
    def update_border(self, borders):
        self.borders = borders