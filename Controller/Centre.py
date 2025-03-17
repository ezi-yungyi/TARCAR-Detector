import websocket

class Centre:
    #SECTION - System Function
    
    def __load_config__(self):
        config_path = os.path.join(self.current_dir, "centre.yaml")

        with open(config_path, "r") as file:
            self.CONFIG = yaml.safe_load(file)
            
        if self.CONFIG is None:
            raise Exception("Config file not found!") 
        
    def __init__(self, name, address, phone, email, website):
        #NOTE - System
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.__load_config__()
        self.capture = cv2.VideoCapture(self.CONFIG['capture']['source'])
        self.lock = threading.Lock()
        
        #NOTE - Veriable
        self.vehicles = []
        self.area_parking_spots = self.get_parking_spots()
    
        #NOTE - Database
        self.db = Database(self.CONFIG['mysql'])
        
        #NOTE -Web Server
        self.web_server_url_prefix = self.CONFIG['webserver']['url']
        self.web_queue = queue.Queue()
        web_thread = threading.Thread(target=self.web_server_thread, daemon=True)
        web_thread.start()

        #NOTE - Communication
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.bind(self.CONFIG['zmq']['router'])
        zmq_thread = threading.Thread(target=self.listen, daemon=True)
        zmq_thread.start()
        
        #NOTE - WebSocket
        self.ws_url = self.CONFIG['websocket']['url']
        self.ws = None
        ws_thread = threading.Thread(target=self.websocket_thread, daemon=True)
        ws_thread.start()

    #SECTION - Centre Function
    
    def vehicle_entry(self, receive_from, track_id, license_plate):
        current_position = {
            "camera_id": receive_from,
            "track_id": track_id
        }
        vehicle = Vehicle(current_position, license_plate)
        self.vehicles.append(vehicle)
        
        for vehicle in self.vehicles:
            if vehicle.verified == False:
                query = f"SELECT COUNT(*) FROM vehicles WHERE license_plate='{vehicle.license_plate}'"
                count = self.db.get_data(query)
                if count == 1:
                    vehicle.verified = True

    def update_vehicle(self, previous_vehicle, current_vehicle, license_plate):
        for vehicle in self.vehicles:
            if vehicle.camera_id == previous_vehicle["camera_id"] and vehicle.track_id == previous_vehicle["track_id"] and vehicle.license_plate == license_plate:
                vehicle.update_location(current_vehicle)
                return

    def update_vehicle_parking_status(self, receive_from, track_id, spot_id, status):
        for vehicle in self.vehicles:
            if vehicle.camera_id == receive_from and vehicle.track_id == track_id:
                if status == "parked":
                    vehicle.set_parked_at(spot_id)
                    all_parking_spots[spot_id].mark_occupied(vehicle.license_plate)
                elif status == "double_parked":
                    vehicle.set_parked_at(spot_id)
                    for a_spot_id in spot_id:
                        all_parking_spots[a_spot_id].mark_double_parked(vehicle.license_plate)
                else:
                    if isinstance(spot_id, list):
                        for a_spot_id in spot_id:
                            all_parking_spots[a_spot_id].mark_double_parked_available(vehicle.license_plate)
                            vehicle.parked_at.remove(a_spot_id)
                    else:
                        all_parking_spots[spot_id].mark_available()
                        vehicle.set_parked_at(-1)
    
    def vehicle_exit(self, receive_from, track_id, license_plate):
        for vehicle in self.vehicles:
            if vehicle.camera_id == receive_from and vehicle.track_id == track_id and vehicle.license_plate == license_plate:
                vehicle.set_parked_at(-1)
                self.vehicles.remove(vehicle)
                return
    
    def get_parking_spots(self, camera_id = None):
        query = f"SELECT id, area_id, camera_id, `index`, position, occupied FROM parking_spots WHERE {f"camera_id={camera_id}" if camera_id is not None else f"area_id={self.CONFIG['area']['id']}"}"
        raw_data = self.db.get_data(query)
        parking_spots = {}
        for row in raw_data:
            spot_id, area_id, camera_id, index, position, occupied = row
            position_load = json.loads(position)
            parking_spots[spot_id] = ParkingSpot(spot_id, area_id, camera_id, index, position_load, occupied)
        
        return parking_spots
    
    def get_borders(self, camera_id):
        query = f"SELECT id, area_id, camera_id, borders FROM areas WHERE camera_id={camera_id}"
        raw_data = self.db.get_data(query)
        borders = {}
        for border_name, border in raw_data.items():
            borders[border_name] = {}
            for line_type, line in border.items():
                borders[border_name][line_type] = {
                    "line_start": np.array(line["line_start"]),
                    "line_end": np.array(line["line_end"])
                }
        
        return borders
            

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
                message_parts = self.socket.recv_multipart()
                if len(message_parts) != 3:
                    print("[ZMQ WARNING] Warning: Received unexpected message format, skipping...")
                    continue
                
                sender_id, target_id, message = message_parts
                
                if target_id != b'centre':
                    self.socket.send_multipart([target_id, message])
                    continue
                
                data = json.loads(message[0].decode())
    
                receive_from = data['from']
                receive_message = data['message']
                    
            except zmq.error.Again as e:
                print(f"[ZMQ ERROR] error: {e}") 
                time.sleep(0.1)

    #!SECTION
    
    #SECTION - Web Server Function
    
    def web_server_thread(self):
        pass
    
    #!SECTION
    
    #SECTION - Web Socket Function
    
    def websocket_thread(self):
        self.ws = websocket.WebSocketApp(
            self.ws_url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )

        self.ws.on_open = self.on_open

        self.ws.run_forever()
    
    def send_heartbeat(self):
        while True:
            time.sleep(60)
            heartbeat_data = {
                "event": "pusher:ping",
                "data": {}
            }
            self.ws.send(json.dumps(heartbeat_data))
            print("Sent heartbeat message")

    def on_message(self, ws, message):  # Added ws argument
        print("Received message:", message)
        try:
            event = json.loads(message)
            
            if event.get('event') == 'pusher:ping':
                print("Received pusher:ping, sending pusher:pong")
                pong_data = {
                    "event": "pusher:pong",
                    "data": {}
                }
                ws.send(json.dumps(pong_data))  # send pong message to maintain connection
        except json.JSONDecodeError:
            print("Error decoding message")

    def on_error(self, ws, error):  # Added ws argument
        print("[WEB SOCKET ERROR]", error)

    def on_close(self, ws, close_status_code, close_msg):  # Added ws argument
        print("Closed connection")

    def on_open(self, ws):  # Corrected to instance method
        print("Connected to WebSocket")
        
        auth_data = {
            "event": "pusher:subscribe",
            "data": {
                "channel": "web-server"  # Subscribe to channel
            }
        }
        ws.send(json.dumps(auth_data))

        # Start heartbeat thread after WebSocket connection is open
        threading.Thread(target=self.send_heartbeat, daemon=True).start()
    
    #!SECTION