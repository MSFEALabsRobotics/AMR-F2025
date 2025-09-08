#below is a small tutorial for time library in python
#write w ros2 node publisher where the robot moves straight for first 5 seconds, the robot rotates for the second 5 seconds, then it stops


import time

# Get the start time (in seconds since epoch)
start_time = time.time()
print("Start time:", start_time)

# Do something (simulate work with sleep)
time.sleep(2)   # wait 2 seconds

# Get current time again
current_time = time.time()
print("Current time:", current_time)

# Calculate elapsed time
elapsed = current_time - start_time
print("Elapsed time:", elapsed, "seconds")

# Compare elapsed time
if elapsed < 5:
    print("Less than 5 seconds passed")
else:
    print("5 or more seconds passed")
