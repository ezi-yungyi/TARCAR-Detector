import cv2
import numpy as np
import time

class TrackedVehicle:
    EXPIRED_GRACE_PERIOD = None
    STOP_DISTANCE_THRESHOLD = None
    STOP_DURATION_THRESHOLD = None
    PARK_INTERSECT_THRESHOLD = None
    DOUPLE_PARK_INTERSECT_THRESHOLD = None
    TOUCH_BORDER_LINE_THRESHOLD = None
    
    @classmethod
    def set_config(cls, config):
        cls.EXPIRED_GRACE_PERIOD = config["expired_grace_period"]
        cls.STOP_DISTANCE_THRESHOLD = config["stop_distance_threshold"]
        cls.STOP_DURATION_THRESHOLD = config["stop_duration_threshold"]
        cls.PARK_INTERSECT_THRESHOLD = config["park_intersect_threshold"]
        cls.DOUPLE_PARK_INTERSECT_THRESHOLD = config["intersect_threshold"]
        cls.TOUCH_BORDER_LINE_THRESHOLD = config["touch_border_line_threshold"]

    def __init__(self, track_id):
        self.track_id = track_id  # Tracked ID
        self.position = None  # (x1, y1, x2, y2)
        self.license_plate = None  # Vehicle licence plate
        
        self.stay_duration = 0  # Vehicle stay duration
        self.first_stop_time = time.time()  # Vehicle first stop time
        self.last_seen_time = time.time()  # Vehicle last seen time
        
        self.parked_at = -1  # Vehicle parked at spot ID
        self.double_parked_at = []  # Vehicle double parked at spot ID
        self.leave_from = None  # Vehicle leave from spot ID
        
        self.border_touch = None
        self.prev_touch_line = None
        
        self.is_inside = False
        self.is_exited = False
        
        self.inside_checked = None
        self.exited_checked = None
        self.parked_checked = None
        self.leave_checked = None               

    #SECTION - Main Functions
    
    def update_position(self, new_position, area_borders, parking_spots):
        movement = np.sum(np.abs(np.array(new_position) - np.array(self.position))) # Calculate movement
        if movement < self.__class__.STOP_DISTANCE_THRESHOLD:
            self.stay_duration = time.time() - self.first_stop_time # Vehicle stay duration
        else:
            self.first_stop_time = time.time() # Reset first stop
            self.stay_duration = 0

        self.position = new_position  # Update position
        
        self.check_is_inside(area_borders)  # Check if vehicle is inside area
        
        if self.is_inside:
            self.check_is_occupied(parking_spots) # Check if vehicle is parked

        self.last_seen_time = time.time()  # Update last seen time
        
    def check_is_occupied(self, parking_spots):
        x1, y1, x2, y2 = self.position
        vehicle_pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        vehicle_area = (x2 - x1) * (y2 - y1)
                    
        occupied_spot = None
        double_parked_spots = []

        for spot in parking_spots:
            spot_pts = np.array([
                [spot.position["FL"]["x"], spot.position["FL"]["y"]],
                [spot.position["FR"]["x"], spot.position["FR"]["y"]],
                [spot.position["BR"]["x"], spot.position["BR"]["y"]],
                [spot.position["BL"]["x"], spot.position["BL"]["y"]]
            ], np.int32)
            spot_area = cv2.contourArea(spot_pts)  
            
            try:
                intersection = cv2.intersectConvexConvex(np.array(spot_pts, np.float32), np.array(vehicle_pts, np.float32))        
                if intersection is not None:
                    intersection_area, _ = intersection
                    if vehicle_area + spot_area - intersection_area > 0:
                        iou = intersection_area / (vehicle_area + spot_area - intersection_area)
                        if iou > self.__class__.PARK_INTERSECT_THRESHOLD:
                            occupied_spot = spot
                    
            except cv2.error:
                print(f"[ERROR] cv2 error in check_is_occupied(): {e}")
                continue
                
            if spot.occupied and (self.is_stopped() or self.is_double_parked()) and not spot.occupied_by == self.track_id:
                dp = {
                    'L': {
                        'x': int(((spot.position["FL"]["x"] - spot.position["BL"]["x"]) * 1/3) + spot.position["FL"]["x"]),
                        'y': int(((spot.position["FL"]["y"] - spot.position["BL"]["y"]) * 1/3) + spot.position["FL"]["y"])
                    },
                    'R': {
                        'x': int(((spot.position["FR"]["x"] - spot.position["BR"]["x"]) * 1/3) + spot.position["FR"]["x"]),
                        'y': int(((spot.position["FR"]["y"] - spot.position["BR"]["y"]) * 1/3) + spot.position["FR"]["y"])
                    }
                }
                double_parked_pts = np.array([
                    [dp["L"]["x"], dp["L"]["y"]],
                    [dp["R"]["x"], dp["R"]["y"]],
                    [spot.position["FR"]["x"], spot.position["FR"]["y"]],
                    [spot.position["FL"]["x"], spot.position["FL"]["y"]],
                ], np.int32)
                dp_area = cv2.contourArea(double_parked_pts)

                try:
                    dp_intersection = cv2.intersectConvexConvex(double_parked_pts.astype(np.float32), vehicle_pts.astype(np.float32))
                    if dp_intersection is not None:
                        dp_intersection_area, _ = dp_intersection

                        if vehicle_area + dp_area - dp_intersection_area > 0:
                            dp_iou = dp_intersection_area / (vehicle_area + dp_area - dp_intersection_area)
                            if dp_iou > self.__class__.DOUPLE_PARK_INTERSECT_THRESHOLD:
                                double_parked_spots.append(spot)
                except cv2.error:
                    print(f"[ERROR] cv2 error in check_is_occupied(): {e}")
                    continue

        if occupied_spot:
            occupied_spot.mark_occupied(self.track_id)
            self.parked_at = occupied_spot.id
            self.leave_checked = False
            self.leave_from = None
        
        if self.parked_at != -1:
            for spot in parking_spots:
                if spot.id == self.parked_at:
                    spot.mark_available()
                    self.leave_from = spot.id
                    self.parked_checked = False
                    self.parked_at = -1
        
        if len(double_parked_spots) != 0 or len(self.double_parked_at) != 0:            
            self.leave_from = []
            for spot in self.__class__.area_parking_spots:
                if not spot.occupied: 
                    continue
                
                if spot.id in double_parked_spots:
                    if spot.id not in self.double_parked_at:
                        if self.is_stopped():
                            spot.mark_double_parked(self.track_id)
                            self.double_parked_at.append(spot.id)
                            self.leave_from = []
                            self.leave_checked = False
                else:
                    if spot.id in self.double_parked_at:
                        spot.mark_double_parked_available(self.track_id)
                        self.leave_from.append(spot.id)
                        self.double_parked_at.remove(spot.id) 
                        self.parked_checked = False
            
    def check_is_inside(self, area_borders):
        for border_name, border in area_borders.items():
            for line_type, line in border.items():
                # Calculate distance from line to center of vehicle
                distance = self._is_touching_line(self.center, line["line_start"], line["line_end"])

                if distance <= self.__class__.TOUCH_BORDER_LINE_THRESHOLD: # If vehicle is close to line
                    if not self.is_inside: # If vehicle is not inside yet
                        if line_type == "inner":
                            self.border_touch = border_name
                            self.prev_touch_line = line_type
                        elif line_type == "outer" and border_name == self.border_touch:
                            self.is_inside = True
                            self.is_exited = False
                            self.inside_checked = False
                            
                    else: # If vehicle is already inside
                        if line_type == "outer":
                            self.border_touch = border_name
                            self.prev_touch_line = line_type
                        elif line_type == "inner" and border_name == self.border_touch:
                            self.is_inside = False
                            self.is_exited = True
                            self.exited_checked = False
    
    #!SECTION
    
    #SECTION - Status Functions
    
    def is_stopped(self):
        return self.stay_duration > self.__class__.STOP_DURATION_THRESHOLD
        
    def is_parked(self):
        return self.is_stopped() and self.parked_at != -1
    
    def is_double_parked(self):
        return len(self.double_parked_at) != 0
    
    def is_leave(self):
        return self.leave_from is not None or []

    def is_expired(self):
        return (time.time() - self.last_seen_time) > self.__class__.EXPIRED_GRACE_PERIOD and not self.is_parked()
    
    #!SECTION
    
    #SECTION - Helper Functions
    
    def _is_touching_line(self, pt, line_start, line_end):
        line = line_end - line_start
        line_length_squared = np.dot(line, line)
        
        if line_length_squared == 0:
            return np.sqrt(np.sum((pt - line_start) ** 2))
        
        t = np.dot(pt - line_start, line) / line_length_squared
        t = max(0, min(1, t))
        projection = line_start + t * line
        
        return np.linalg.norm(pt - projection)
    
    #!SECTION