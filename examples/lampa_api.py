from lampa_fleet import Fleet, Robot
import time

fleet = Fleet()

turtle3 = fleet.add_robot("turtle3")

while True:
    turtle3.set_cmd_vel(0.0, -0.5)
    print(turtle3.pose)
    time.sleep(0.01)