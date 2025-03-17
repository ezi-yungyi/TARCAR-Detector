import mysql.connector

class Database:
    def __init__(self, config):
        self.config = config
        
    def connect_db(self):
        self.conn = mysql.connector.connect(
            host=self.config["host"],
            user=self.config["user"],
            password=self.config["password"],
            database=self.config["database"],
        )
        self.cursor = self.conn.cursor()
        
    def close_db(self):
        self.cursor.close()
        self.conn.close()

    def get_data(self, query):
        self.connect_db()
        self.cursor.execute(query)
        result = self.cursor.fetchall()
        self.close_db()
        return result
    
    def insert_data(self, query):
        self.connect_db()
        self.cursor.execute(query)
        self.conn.commit()
        self.close_db()