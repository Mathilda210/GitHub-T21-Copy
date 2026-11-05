"""Petit script partagé par ana pour tenter de recupérer 
les données caméras comme elle fait avec sa realsense directement
--> echec"""

import pyrealsense2 as rs
import time
 
pipeline = rs.pipeline()
config = rs.config()
config.enable_record_to_file("capture.db3")
pipeline.start(config)
time.sleep(10)
pipeline.stop()