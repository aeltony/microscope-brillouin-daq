from TestDevice import TestDevice

import threading
from time import sleep
from timeit import default_timer

stop_event = threading.Event()

testDevice1 = TestDevice(stop_event, 0.01, 1)
testDevice2 = TestDevice(stop_event, 0.1, 2)

# Start the devices in their own thread
testDevice1.start()
testDevice2.start()

sleep(1)

sequentialAcqList = [testDevice2]

# first turn off free running mode
for dev in sequentialAcqList:
	dev.pause()	
	dev.runMode = 1

for dev in sequentialAcqList:
	dev.unpause()

# provide scan settings
startTime = default_timer()      

print "Starting Scan"
                
for k in range(100):  
	print "scan loop %d" % k

	# loopStartTime = default_timer() 
	for dev in sequentialAcqList:
		dev.continueEvent.set()

	# sleep(0.5)

	for dev in sequentialAcqList:
		dev.completeEvent.wait()
		dev.completeEvent.clear()
	# loopEndTime = default_timer()
	# print loopEndTime-loopStartTime

	# sleep(0.1)

endTime = default_timer()
print "scan acq time = %.3f" % (endTime - startTime)

stop_event.set()
sleep(0.5)