import csv
import time
from datetime import datetime
import os

class CmdVelLogger:
    def __init__(self, log_dir="logs", filename_prefix="deploy_log"):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{filename_prefix}_{timestamp}.csv"
        self.filepath = os.path.join(log_dir, self.filename)
        
        self.csv_file = None
        self.csv_writer = None
        self._initialize_csv()
    
    def _initialize_csv(self):
        self.csv_file = open(self.filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'cmd_vel_x', 'cmd_vel_y', 'cmd_vel_yaw'])
        self.csv_file.flush()
    
    def log_cmd_vel(self, cmd_vel_x, cmd_vel_y, cmd_vel_yaw):
        timestamp = time.time()
        self.csv_writer.writerow([timestamp, cmd_vel_x, cmd_vel_y, cmd_vel_yaw])
        self.csv_file.flush()
    
    def close(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None