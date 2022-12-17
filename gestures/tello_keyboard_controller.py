# from djitellopy import Tello


class TelloKeyboardController:
    def __init__(self, tello):
        self.tello = tello

    def control(self, key):
        self.tello.speed.linear.x = 0.0
        self.tello.speed.linear.y = 0.0
        self.tello.speed.linear.z = 0.0
        self.tello.speed.angular.z = 0.0

        if key == ord('w'):
            self.tello.speed.linear.x = 0.3
            # self.tello.move_forward(30)
        elif key == ord('s'):
            self.tello.speed.linear.x = -0.3
            # self.tello.move_back(30)
        elif key == ord('a'):
            self.tello.speed.linear.y = 0.3
            # self.tello.move_left(30)
        elif key == ord('d'):
            self.tello.speed.linear.y = -0.3

            # self.tello.move_right(30)
        elif key == ord('e'):
            self.tello.speed.angular.z = -0.3

            # self.tello.rotate_clockwise(30)
        elif key == ord('q'):
            self.tello.speed.angular.z = 0.3

            # self.tello.rotate_counter_clockwise(30)
        elif key == ord('r'):
            self.tello.speed.linear.z = 0.3

            # self.tello.move_up(30)
        elif key == ord('f'):
            self.tello.speed.linear.z = -0.3

            # self.tello.move_down(30)
        self.tello.pub_vel.publish(self.tello.speed)
