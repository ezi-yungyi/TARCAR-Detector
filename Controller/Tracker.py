import os
import time
import yaml
import numpy as np
import cv2
from ParkingSpot import ParkingSpot
from TrackedVehicle import TrackedVehicle
from Database import Database
from Area import Area
from ultralytics import YOLO
from queue import Queue
import zmq
import json
import threading
from zmq.error import Again  # Add this import
import requests

class Tracker:
    #SECTION - System Function
    
    def __load_config__(self):
        config_path = os.path.join(self.current_dir, "tracker1.yaml")

        with open(config_path, "r") as file:
            self.CONFIG = yaml.safe_load(file)
            
        if self.CONFIG is None:
            raise Exception("Config file not found!") 
    
    def __init__(self):
        #NOTE - System
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.__load_config__()
        self.capture = cv2.VideoCapture(self.CONFIG['capture']['source'])
        self.lock = threading.Lock()
        
        #NOTE - Variable
        self.incoming_vehicles = {}
        self.tracked_vehicles = {}
        self.frame = None
        self.area = Area()
        
        #NOTE - Detection Model
        self.MODEL = YOLO(model=self.CONFIG['detection']['model'], verbose=False)
        self.DETECT_CLASSES = self.CONFIG['detection']['classes']
        
        #NOTE - Database
        self.db = Database(self.CONFIG['mysql'])

        #NOTE - Communication
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt_string(zmq.IDENTITY, f"camera-{self.CONFIG['area']['camera_id']}")
        self.socket.connect(self.CONFIG['zmq']['router'])
        zmq_thread = threading.Thread(target=self.listen, daemon=True)
        zmq_thread.start()
        
        #NOTE - Set Config for Tracked Vehicle
        TrackedVehicle.set_config(self.CONFIG['tracked_vehicle'])
        
        while not self.area.borders or not self.area.parking_spots:
            self.send('centre', json.dumps({
                "from": self.CONFIG['area']['camera_id'],
                "function": "RequestInitialize",
            }))
            time.sleep(5) # Wait for 5 seconds
            continue
    
    #!SECTION
            
    #SECTION - Main Function
    
    def detect_vehicles(self):
        results = self.MODEL.track(self.frame, persist=True, conf=0.5)
        detections = results[0]
        
        if detections.boxes:
            for box in detections.boxes:
                
                class_id = int(box.cls[0])
                
                if class_id in self.DETECT_CLASSES:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    track_id = int(box.id[0]) if box.id is not None else -1
                    
                    # Check if vehicle is already tracked
                    if track_id not in self.tracked_vehicles:
                        self.tracked_vehicles[track_id] = TrackedVehicle(track_id)
                        
                    current_vehicle = self.tracked_vehicles[track_id]
                    
                    current_vehicle.update_position((x1, y1, x2, y2))
                    
                    target = None
                    data = {
                        "from": self.CONFIG['area']['camera_id'],
                    }
                    
                    if current_vehicle.is_inside and not current_vehicle.inside_checked:
                        current_vehicle.inside_checked = True
                        incoming_vehicle = self.incoming_vehicles[current_vehicle.border_touch].get() if not self.incoming_vehicles[current_vehicle.border_touch].empty() else None
                        current_vehicle.license_plate = incoming_vehicle['license_plate'] if incoming_vehicle else None
                        
                        target = 'centre'
                        data.update({
                            "function": "update_vehicle",
                            "previous_vehicle": {
                                "camera_id": incoming_vehicle['camera_id'] if previous_vehicle else None,
                                "track_id": incoming_vehicle['track_id'] if previous_vehicle else None,
                            },
                            "current_vehicle": {
                                "camera_id": self.CONFIG['area']['camera_id'],
                                "track_id": track_id,
                            },
                            "license_plate": current_vehicle.license_plate
                        })
                        
                    if current_vehicle.is_parked() and not current_vehicle.parked_checked:
                        current_vehicle.parked_checked = True
                        target = "centre"
                        data.update({
                            "function": "update_vehicle_parking",
                            "track_id": track_id,
                            "spot_id": current_vehicle.parked_at,
                            "status": 'parked',
                        })
                        
                    if current_vehicle.is_double_parked() and not current_vehicle.parked_checked:
                        current_vehicle.parked_checked = True
                        target = "centre"
                        data.update({
                            "function": "update_vehicle_parking",
                            "track_id": track_id,
                            "spot_id": current_vehicle.double_parked_at,
                            "status": 'double_parked',
                        })
                        
                    if current_vehicle.is_leave() and not current_vehicle.leave_checked:
                        current_vehicle.leave_checked = True
                        target = "centre"
                        data.update({
                            "function": "update_vehicle_parking",
                            "track_id": track_id,
                            "spot_id": current_vehicle.leave_from,
                            "status": 'leave',
                        })
                        
                    if current_vehicle.is_exited and not current_vehicle.exited_checked:
                        current_vehicle.exited_checked = True
                        next_camera = self.CONFIG['area']['next_camera'][current_vehicle.border_touch]
                        target, function = f"camera-{next_camera}", "vehicle_to_next" if next_camera != -1 else "centre", "vehicle_exit"
                        data.update({
                            "function": function,
                            "track_id": track_id,
                            "license_plate": current_vehicle.license_plate,
                        })
                        
                    self.send(target, json.dumps(data))

                    text = f"ID {current_vehicle.track_id} - {current_vehicle.license_plate}"
                    color = (0, 0, 255) if current_vehicle.is_parked() or current_vehicle.is_double_parked() else (0, 255, 0)
                    
                    self.drawRectangles(x1, y1, x2, y2, text, color, 2)
                    self.drawPoint(current_vehicle.center[0], current_vehicle.center[1], color)
    
    #!SECTION
    
    #SECTION - Display Helper Function    
    
    def drawPolylines(self, pts, text = '', color=(0, 255, 0), thickness=2):
        pts = pts.reshape((-1, 1, 2))
        x, y = pts[0][0]
        cv2.polylines(self.frame, [pts], isClosed=True, color=color, thickness=thickness)
        cv2.putText(self.frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    def drawRectangles(self, x1, y1, x2, y2, text='', color=(0, 255, 0), thickness=2):
        cv2.rectangle(self.frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(self.frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    def drawPoint(self, x, y, color=(0, 255, 0), radius=2):
        cv2.circle(self.frame, (x, y), radius, color, -1)
    
    #!SECTION
    
    #SECTION - Communication Function
    
    def send(self, target_id, message):
        try:
            self.socket.send_multipart([target_id.encode(), message.encode()])
        except zmq.ZMQError as e:
            print(f"[ZMQ ERROR] error: {e}") 
        
    def listen(self):
        while True:
            try:
                message = self.socket.recv_multipart()
                data = json.loads(message[0].decode())
    
                receive_from = data['from']
                if receive_from == 'centre':
                    
                    if data['function'] == 'Testing':
                        print("[ZMQ] Testing")
                        continue
                    
                    elif data['function'] == 'Initialize':
                        force = data['force'] if 'force' in data else False
                        receive_data = data['data'] if 'data' in data else None
                        
                        self.update_parking_spots(force, receive_data['parking_spots'])
                        self.update_area_borders(force, receive_data['borders'])
                        
                    elif data['function'] == 'ping':
                        target = 'centre'
                        data = json.dumps({
                            "from": self.CONFIG['area']['camera_id'],
                            "reply": 'pong',
                        })
                        self.send(target, data)
                        
                    elif data['function'] == 'take_picture':
                        target = 'centre'
                        area_picture = self.take_picture()
                        data = json.dumps({
                            "from": self.CONFIG['area']['camera_id'],
                            "picture": area_picture,
                        })
                        self.send(target, data)
                    
                    elif data['function'] == 'update_parking':
                        force = data['force'] if 'force' in data else False
                        receive_data = data['data'] if 'data' in data else None
                        
                        self.update_parking_spots(force, receive_data['parking_spots'])
                        
                    elif data['function'] == 'update_border':
                        force = data['force'] if 'force' in data else False
                        receive_data = data['data'] if 'data' in data else None
                        
                        self.update_area_borders(force, receive_data['borders'])
                    
                else:
                    if data['function'] == 'vehicle_to_next':
                        track_id = data['track_id']
                        source_direction = None
                        
                        # Check if vehicle is inside area
                        for direction, cam_id in self.CONFIG['area']['next_camera'].items():
                            if cam_id == receive_from:
                                self.incoming_vehicles[direction].put({"camera_id": receive_from, "track_id": track_id, "license_plate": data['license_plate']})
                                break 
                    
            except zmq.error.Again as e:
                print(f"[ZMQ ERROR] error: {e}") 
                time.sleep(0.1)
    
    #!SECTION
    
    #SECTION - Edge Function
    
    def take_picture(self):
        frame = None
        if (self.frame is not None):
            frame = self.frame
            cv2.imwrite(f"camera_id-{self.CONFIG['area']['camera_id']}.jpg", self.frame)
        else:
            ret, current_frame = self.capture.read()
            if ret:
                frame = current_frame
        
        return frame
    
    def update_parking_spots(self, force = False, receive_data = None):
        with self.lock:
            has_parked_vehicles = any(vehicle.is_parked for vehicle in self.tracked_vehicles.values())
            if has_parked_vehicles and not force:
                print("[ERROR] Cannot update parking spots while vehicles are parked inside area")
                return
            
            parking_spot_raw_data = receive_data
            parking_spots = []
            
            for row in parking_spot_raw_data:
                spot_id, area_id, camera_id, index, position, occupied = row
                position_load = json.loads(position)
                for key in position_load:
                    position_load[key]['x'] = position_load[key]['x']
                    position_load[key]['y'] = position_load[key]['y']
                
                parking_spots.append(ParkingSpot(spot_id, area_id, camera_id, index, position_load, occupied))
                
            self.area.update_parking_spots(parking_spots)
        
    def update_area_borders(self, force = False, receive_data = None):
        with self.lock:
            if any(not queue.empty() for queue in self.incoming_vehicles.values()) and not force:
                print("[ERROR] Cannot update area borders while vehicles are inside area")
                return

            border_raw_data = receive_data
            borders = {}
            
            for border_name, border in border_raw_data.items():
                borders[border_name] = {}
                for line_type, line in border.items():
                    borders[border_name][line_type] = {
                        "line_start": np.array(line["line_start"]),
                        "line_end": np.array(line["line_end"])
                    }
                    
            self.incoming_vehicles = {border_name: Queue() for border_name in borders}
            self.area.update_border(borders)
            
    #!SECTION
    
    #SECTION - Run Function
    
    def run(self):
        while self.capture.isOpened():
            ret, self.frame = self.capture.read()
            if not ret:
                break
            
            #NOTE - Run Detection
            self.detect_vehicles()
            
            #NOTE - Check if vehicle is expired
            expired_vehicles = [track_id for track_id in list(self.tracked_vehicles.keys()) if self.tracked_vehicles[track_id].is_expired()]
            for track_id in expired_vehicles:
                del self.tracked_vehicles[track_id]
            
            #NOTE - Display Parking Spots and Borders
            for spot in self.area.parking_spots:
                spot_pts = np.array([
                    [spot.position["FL"]["x"], spot.position["FL"]["y"]],
                    [spot.position["FR"]["x"], spot.position["FR"]["y"]],
                    [spot.position["BR"]["x"], spot.position["BR"]["y"]],
                    [spot.position["BL"]["x"], spot.position["BL"]["y"]]
                ], np.int32)
                
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
                
                self.drawPolylines(double_parked_pts, color=(0, 0, 255) if len(spot.double_parked_by) != 0 else (255, 255, 0))
                self.drawPolylines(spot_pts, color=(0, 0, 255) if spot.occupied else (0, 255, 0))
                for label, coords in spot.position.items():
                        x, y = coords["x"], coords["y"]
                        cv2.putText(self.frame, label, (int(x), int(y) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            #NOTE - Display Borders        
            for border_name, border in self.area.borders.items():
                self.drawPolylines(np.array([border["inner"]["line_start"], border["inner"]["line_end"]]), color=(0, 255, 0))
                self.drawPolylines(np.array([border["outer"]["line_start"], border["outer"]["line_end"]]), color=(0, 0, 255))
                       
            cv2.imshow("TARCAR", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.capture.release()
        cv2.destroyAllWindows()
        
    #!SECTION
        
if __name__ == "__main__":
    tracker = EdgeTracker()
    tracker.run()