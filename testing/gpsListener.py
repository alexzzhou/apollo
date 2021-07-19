from gps import * 
import time

gpsd = gps(mode = WATCH_ENABLE | WATCH_NEWSTYLE, port = 9000)

while True: 
    data = gpsd.next()
    
        #print(str(getattr(data, 'lat', 'unknown')))
    print(data)
