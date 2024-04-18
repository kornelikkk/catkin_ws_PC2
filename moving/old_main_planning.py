#!/usr/bin/env python3

#import sys
from enum import Enum
from time import sleep
from typing import NamedTuple

import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class Point(NamedTuple):
    pos_x: int
    pos_y: int


class RuleTopic(Enum):
    RIGHT = 'RIGHT'
    LEFT = 'LEFT'
    FORWARD = 'FORWARD'
    NO = 'NO'


class Direction(Enum):
    up = 1
    right = 2
    down = 3
    left = 4

    def turn_right(self):
        cls = self.__class__
        members = list(cls)
        ind = members.index(self) + 1
        if ind >= len(members):
            ind = 0
        return members[ind]

    def turn_left(self):
        cls = self.__class__
        members = list(cls)
        ind = members.index(self) - 1
        if ind < 0:
            ind = len(members) - 1
        return members[ind]

    def reverse(self):
        cls = self.__class__
        members = list(cls)
        ind = (members.index(self) + 2) % len(members)
        return members[ind]

DIR_TO_ANGLE = {
    Direction.up: "0", 
    Direction.right: "-90",
    Direction.down: "180",
    Direction.left: "90"
}

NEAR_DICT = {
    Direction.up: Point(0, 1), 
    Direction.right: Point(1, 0),
    Direction.down: Point(0, -1),
    Direction.left: Point(-1, 0)
}

DIR_BY_COORD_DIFF = {NEAR_DICT[k]: k for k in NEAR_DICT}


class Robot:
    def __init__(self, start_coord: Point = Point(0, 0), end_coord: Point = Point(0, 0)) -> None:
        self.direct: Direction = Direction.up
        self.pos_x: int = start_coord.pos_x
        self.pos_y: int = start_coord.pos_y

        self.is_moving = 'False'
        self.map_list = []

        self.pub_angle = None
        self.pub_move = "False"
        self.rate = None
        self.sub_detect = None

        self.detected = 'NO'

    def get_coord(self):
        return Point(self.pos_x, self.pos_y)

    def callback_detect(self, msg):
        self.detected = msg.data
        print("detected sign ", self.detected)

    def callback_is_moving(self, msg):
        self.is_moving = msg.data
        print("is_moving ", self.is_moving)

    def r_init_node(self):
        self.pub_angle = rospy.Publisher('/angle', String, queue_size=10)

        self.sub_detect = rospy.Subscriber('/detect_result', String, self.callback_detect)

        self.pub_move = rospy.Publisher('/move_forward', String, queue_size=10)
        self.is_moving = rospy.Subscriber('/move_forward', String, self.callback_is_moving)

        rospy.init_node('angle_move_talker', anonymous=True)
        self.rate = rospy.Rate(10)

    def turn(self, degree: int = 90) -> None:
        if degree > 0:
            for _ in range(degree // 90):
                self.direct = self.direct.turn_right()

        elif degree < 0:
            for _ in range(-degree // 90):
                self.direct = self.direct.turn_left()


        position = DIR_TO_ANGLE[self.direct]
        rospy.loginfo(position)
        self.pub_angle.publish(position)
        self.rate.sleep()


    def move_forward(self) -> None:
        if self.direct == Direction.up:
            self.pos_y += 1
        elif self.direct == Direction.right:
            self.pos_x += 1
        elif self.direct == Direction.down:
            self.pos_y -= 1
        elif self.direct == Direction.left:
            self.pos_x -= 1

        self.pub_move.publish('True')
        self.rate.sleep()


    def next_move(self, old_coord: Point, new_coord: Point) -> None:
        coord_diff = Point(
            new_coord.pos_x - old_coord.pos_x,
            new_coord.pos_y - old_coord.pos_y
        )

        new_dir = DIR_BY_COORD_DIFF[coord_diff]

        if self.direct != new_dir:
            how_turn = {
                self.direct.turn_right(): 90,
                self.direct.turn_left(): -90,
                self.direct.reverse(): 180
            }
            self.turn(how_turn[new_dir])
            sleep(15)

        self.move_forward()

    # def scan_cell(self) -> dict:
    #     scan_data = {}

    #     coord = Point(self.pos_x, self.pos_y)
    #     for dir, point in NEAR_DICT.items():
    #         sub_coord = Point(
    #             coord.pos_x + point.pos_x,
    #             coord.pos_y + point.pos_y
    #         )

    #         #sub_cell = self.real_map.cell_by_coord.get(sub_coord)
    #         if sub_cell is not None:
    #             # Получаем правила поворота из соседней клетки для нашей стороны
    #             rule_from_near = sub_cell.turn_by_near_coord[dir.reverse()]
    #             scan_data[dir] = rule_from_near

    #     return scan_data

    # TODO Тут проверка на аруку
    def check_finish(self) -> bool:
        # coord = Point(self.pos_x, self.pos_y)
        # cell = self.real_map.cell_by_coord[coord]
        # return cell.is_finish
        return False


class PathFinder:
    def __init__(self, start_coord: Point = Point(0, 0), end_coord: Point = Point(0, 0)) -> None:
        self.robot = Robot(start_coord, end_coord)
        self.current_point = Point(0, 0)
        
        #self.is_moving = "False"

    def move_to(self, new_coord: Point) -> None:
        old_coord = self.current_point
        self.robot.next_move(old_coord, new_coord)

    def route_planning(self):

        topic_msg = self.robot.detected # TODO читаем топик распознования знаков
        turn_rule = RuleTopic[topic_msg]
        print(topic_msg)
        direct = self.robot.direct
        coord = self.current_point

        if turn_rule == RuleTopic.FORWARD or turn_rule == RuleTopic.NO:
            sub_coord = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord)

        elif turn_rule == RuleTopic.RIGHT:
            sub_coord1 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            direct = direct.turn_right()
            sub_coord2 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord1)
            self.robot.map_list.append(sub_coord2)

        elif turn_rule == RuleTopic.LEFT:
            sub_coord1 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            direct = direct.turn_left()
            sub_coord2 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord1)
            self.robot.map_list.append(sub_coord2)

    def start(self):
        while not rospy.is_shutdown():
            if self.robot.is_moving == "True": # TODO топик чтение
                sleep(1)
                continue

            if self.robot.check_finish():
                print(f'FINISH COORD = {self.current_point}')
                break

            map_list = self.robot.map_list
            if not map_list:
                sleep(1)
                self.route_planning()

            while map_list:
                print(f"{map_list=}")
                next_coord = map_list.pop(0)
                self.move_to(next_coord)
                self.robot.is_moving = "True" # TODO топик запись
                print(next_coord)
                self.current_point = self.robot.get_coord()


def main():
    path_finder = PathFinder()
    path_finder.robot.r_init_node()
    sleep(7)
    path_finder.start()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

#import sys
from enum import Enum
from time import sleep
from typing import NamedTuple

import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class Point(NamedTuple):
    pos_x: int
    pos_y: int


class RuleTopic(Enum):
    RIGHT = 'RIGHT'
    LEFT = 'LEFT'
    FORWARD = 'FORWARD'
    NO = 'NO'


class Direction(Enum):
    up = 1
    right = 2
    down = 3
    left = 4

    def turn_right(self):
        cls = self.__class__
        members = list(cls)
        ind = members.index(self) + 1
        if ind >= len(members):
            ind = 0
        return members[ind]

    def turn_left(self):
        cls = self.__class__
        members = list(cls)
        ind = members.index(self) - 1
        if ind < 0:
            ind = len(members) - 1
        return members[ind]

    def reverse(self):
        cls = self.__class__
        members = list(cls)
        ind = (members.index(self) + 2) % len(members)
        return members[ind]

DIR_TO_ANGLE = {
    Direction.up: "0", 
    Direction.right: "-90",
    Direction.down: "180",
    Direction.left: "90"
}

NEAR_DICT = {
    Direction.up: Point(0, 1), 
    Direction.right: Point(1, 0),
    Direction.down: Point(0, -1),
    Direction.left: Point(-1, 0)
}

DIR_BY_COORD_DIFF = {NEAR_DICT[k]: k for k in NEAR_DICT}


class Robot:
    def __init__(self, start_coord: Point = Point(0, 0), end_coord: Point = Point(0, 0)) -> None:
        self.direct: Direction = Direction.up
        self.pos_x: int = start_coord.pos_x
        self.pos_y: int = start_coord.pos_y

        self.is_moving = 'False'
        self.map_list = []

        self.pub_angle = None
        self.pub_move = "False"
        self.rate = None
        self.sub_detect = None

        self.detected = 'NO'

    def get_coord(self):
        return Point(self.pos_x, self.pos_y)

    def callback_detect(self, msg):
        self.detected = msg.data
        print("detected sign ", self.detected)

    def callback_is_moving(self, msg):
        self.is_moving = msg.data
        print("is_moving ", self.is_moving)

    def r_init_node(self):
        self.pub_angle = rospy.Publisher('/angle', String, queue_size=10)

        self.sub_detect = rospy.Subscriber('/detect_result', String, self.callback_detect)

        self.pub_move = rospy.Publisher('/move_forward', String, queue_size=10)
        self.is_moving = rospy.Subscriber('/move_forward', String, self.callback_is_moving)

        rospy.init_node('angle_move_talker', anonymous=True)
        self.rate = rospy.Rate(10)

    def turn(self, degree: int = 90) -> None:
        if degree > 0:
            for _ in range(degree // 90):
                self.direct = self.direct.turn_right()

        elif degree < 0:
            for _ in range(-degree // 90):
                self.direct = self.direct.turn_left()


        position = DIR_TO_ANGLE[self.direct]
        rospy.loginfo(position)
        self.pub_angle.publish(position)
        self.rate.sleep()


    def move_forward(self) -> None:
        if self.direct == Direction.up:
            self.pos_y += 1
        elif self.direct == Direction.right:
            self.pos_x += 1
        elif self.direct == Direction.down:
            self.pos_y -= 1
        elif self.direct == Direction.left:
            self.pos_x -= 1

        self.pub_move.publish('True')
        self.rate.sleep()


    def next_move(self, old_coord: Point, new_coord: Point) -> None:
        coord_diff = Point(
            new_coord.pos_x - old_coord.pos_x,
            new_coord.pos_y - old_coord.pos_y
        )

        new_dir = DIR_BY_COORD_DIFF[coord_diff]

        if self.direct != new_dir:
            how_turn = {
                self.direct.turn_right(): 90,
                self.direct.turn_left(): -90,
                self.direct.reverse(): 180
            }
            self.turn(how_turn[new_dir])
            sleep(15)

        self.move_forward()

    # def scan_cell(self) -> dict:
    #     scan_data = {}

    #     coord = Point(self.pos_x, self.pos_y)
    #     for dir, point in NEAR_DICT.items():
    #         sub_coord = Point(
    #             coord.pos_x + point.pos_x,
    #             coord.pos_y + point.pos_y
    #         )

    #         #sub_cell = self.real_map.cell_by_coord.get(sub_coord)
    #         if sub_cell is not None:
    #             # Получаем правила поворота из соседней клетки для нашей стороны
    #             rule_from_near = sub_cell.turn_by_near_coord[dir.reverse()]
    #             scan_data[dir] = rule_from_near

    #     return scan_data

    # TODO Тут проверка на аруку
    def check_finish(self) -> bool:
        # coord = Point(self.pos_x, self.pos_y)
        # cell = self.real_map.cell_by_coord[coord]
        # return cell.is_finish
        return False


class PathFinder:
    def __init__(self, start_coord: Point = Point(0, 0), end_coord: Point = Point(0, 0)) -> None:
        self.robot = Robot(start_coord, end_coord)
        self.current_point = Point(0, 0)
        
        #self.is_moving = "False"

    def move_to(self, new_coord: Point) -> None:
        old_coord = self.current_point
        self.robot.next_move(old_coord, new_coord)

    def route_planning(self):

        topic_msg = self.robot.detected # TODO читаем топик распознования знаков
        turn_rule = RuleTopic[topic_msg]
        print(topic_msg)
        direct = self.robot.direct
        coord = self.current_point

        if turn_rule == RuleTopic.FORWARD or turn_rule == RuleTopic.NO:
            sub_coord = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord)

        elif turn_rule == RuleTopic.RIGHT:
            sub_coord1 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            direct = direct.turn_right()
            sub_coord2 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord1)
            self.robot.map_list.append(sub_coord2)

        elif turn_rule == RuleTopic.LEFT:
            sub_coord1 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            direct = direct.turn_left()
            sub_coord2 = Point(
                coord.pos_x + NEAR_DICT[direct].pos_x,
                coord.pos_y + NEAR_DICT[direct].pos_y
            )
            self.robot.map_list.append(sub_coord1)
            self.robot.map_list.append(sub_coord2)

    def start(self):
        while not rospy.is_shutdown():
            if self.robot.is_moving == "True": # TODO топик чтение
                sleep(1)
                continue

            if self.robot.check_finish():
                print(f'FINISH COORD = {self.current_point}')
                break

            map_list = self.robot.map_list
            if not map_list:
                sleep(1)
                self.route_planning()

            while map_list:
                print(f"{map_list=}")
                next_coord = map_list.pop(0)
                self.move_to(next_coord)
                self.robot.is_moving = "True" # TODO топик запись
                print(next_coord)
                self.current_point = self.robot.get_coord()


def main():
    path_finder = PathFinder()
    path_finder.robot.r_init_node()
    sleep(7)
    path_finder.start()


if __name__ == '__main__':
    main()
