class ParkingSpot:
    def __init__(self, id, area_id, camera_id, index, position, occupied):
        self.id = id
        self.index = index
        self.area_id = area_id
        self.camera_id = camera_id
        self.position = eval(position) if isinstance(position, str) else position
        self.occupied = occupied
        self.occupied_by = None
        self.double_parked_by = []

    def mark_occupied(self, occupied_by):
        """MARK AS OCCUPIED"""
        self.occupied = True
        self.occupied_by = occupied_by

    def mark_available(self):
        """MARK AS AVAILABLE"""
        self.occupied = False
        self.occupied_by = None
        
    def mark_double_parked(self, double_parked_by):
        """MARK AS DOUBLE PARKED"""
        if double_parked_by not in self.double_parked_by:
            self.double_parked_by.append(double_parked_by)
        
    def mark_double_parked_available(self, double_parked_by):
        """MARK AS DOUBLE PARKED AVAILABLE"""
        if double_parked_by in self.double_parked_by:
            self.double_parked_by.remove(double_parked_by)
        