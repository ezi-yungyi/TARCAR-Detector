import cv2
import numpy as np
import time

class Vehicle:
    def __init__(self, current_position = {camera_id: None, track_id: None}, license_plate = None):
        self.track_id = current_position["track_id"]
        self.camera_id = current_position["camera_id"]
        self.license_plate = license_plate
        self.parked_at = -1
        
        self.verified = False
        self.last_update = time.time()
        
    def update_location(self, current_position):
        self.track_id = current_position["track_id"]
        self.camera_id = current_position["camera_id"]
        self.last_update = time.time()
        
    def set_license_plate(self, license_plate):
        self.license_plate = license_plate
    
    def set_parked_at(self, spot_id):
        self.parked_at = spot_id