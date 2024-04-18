#!/usr/bin/env python3
import sys
from enum import Enum
from time import sleep
from typing import NamedTuple

import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class Point(NamedTuple):
    pos_x: int
    pos_y: int


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

#TurnRule: TypeAlias = dict[Direction, list[Direction]]

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
        # Скорее всего координаты тоже выкидываем, т.к. они уже учитываются в pathfinder
        self.pos_x: int = start_coord.pos_x
        self.pos_y: int = start_coord.pos_y
        # self.real_map: RealMap = RealMap(test_map, end_coord) # ЗАГЛУШКА

        self.pub_angle = None
        self.pub_move = None
        self.rate = None

    def r_init_node(self):
        self.pub_angle = rospy.Publisher('/angle', String, queue_size=10)
        self.pub_move = rospy.Publisher('/move_forward', String, queue_size=10)
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

        self.pub_move.publish("True")
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

    # def check_finish(self) -> bool:
    #     coord = Point(self.pos_x, self.pos_y)
    #     cell = self.real_map.cell_by_coord[coord]
    #     return cell.is_finish


class Cell:
    def __init__(self, x=0, y=0) -> None:
        self.pos_x: int = x
        self.pos_y: int = y
        self.near_dict: dict[Direction, Cell] = {}
        self.turn_by_near_coord: dict[Direction, list[Direction]] = {}

    # rule = {
    #     Direction.up: [Direction.left, Direction.right],
    #     Direction.left: [Direction.up],
    #     Direction.right: [Direction.left]
    # }
    def update_turn_rule(self, new_turn_rule: dict) -> None:
        for dir, new_connect in new_turn_rule.items():
            self.turn_by_near_coord[dir] = new_connect

    def get_near_dict(self) -> dict:
        return self.near_dict

    def get_coord(self) -> Point:
        return Point(self.pos_x, self.pos_y)


class Mode(Enum):
    scan = 'scan'
    back = 'back'


class EdgeDir(Enum):
    out = 'out'
    input = 'input'


class Edge:
    def __init__(self, start_node, start_dir, end_node, end_dir, cell_list) -> None:
        self.start_node: Node = start_node
        self.start_dir: Direction = start_dir
        self.end_node: Node = end_node
        self.end_dir: Direction = end_dir
        self.cell_list: list[Cell] = cell_list

    def get_value(self):
        return len(self.cell_list)


class Node:
    def __init__(self, cell: Cell) -> None:
        self.coord: Point = cell.get_coord()
        self.cell = cell
        self.open_direction: list[Direction] = (cell.get_near_dict().keys())
        self.edge_by_direction: dict[(Direction, EdgeDir), Edge] = {}

    def add_edge(self, direction: Direction, edge_dir: EdgeDir, edge: Edge):
        self.edge_by_direction[(direction, edge_dir)] = edge

    def get_coord(self):
        return self.coord


class PathFinder:
    def __init__(self, start_coord: Point = Point(0, 0), end_coord: Point = Point(0, 0)) -> None:
        # start_coord и end_coord - ЗАГЛШУКА для симуляции
        self.robot = Robot(start_coord, end_coord)
        #self.current_cell = Cell(0, 0)
        self.current_point = Point(0, 0)
        # self.last_cell = self.current_cell
        # self.open_cell_list: list[Point] = [Point(0, 0)]
        # self.close_cell_list: list[Point] = []
        # self.cell_by_coord: dict[Point, Cell] = {Point(0, 0): self.current_cell}
        # self.back_path = None

        # self.last_node: Node = Node(self.current_cell)
        # self.last_node_dir: Direction = None
        # self.node_by_coord: dict[Point, Node] = {Point(0, 0): self.last_node}
        # self.current_edge: list[Point] = []

        # Дописать алгоритм Дейкстры с учетом правил на повороты
        '''
        Перекрестки и тупики - вершины графа со списком смежных сторон и правилами куда откуда можно повернуть
        Ребро - линия между 2мя парами из узла и стороны
        edge = {
            (
                (Node_1, Direction.up),
                (Node_2, Direction.left) # путь из верха 1го перекрестка в левую часть 2го, по сути поворот буквой Г
            ): [1, 2, 3 ...] # список координат клеток пути
        }
        '''

    def move_to(self, new_coord: Point) -> None:
        #self.last_cell = self.current_cell
        #self.current_cell = self.cell_by_coord[new_coord]
        old_coord = self.current_point

        # При выходе из узла
        # last_node = self.node_by_coord.get(self.last_cell.get_coord())
        # if last_node is not None:
        #     coord_diff = Point(
        #         new_coord.pos_x - old_coord.pos_x,
        #         new_coord.pos_y - old_coord.pos_y
        #     )
        #     self.last_node = last_node
            #self.last_node_dir = (DIR_BY_COORD_DIFF[coord_diff])

        self.robot.next_move(old_coord, new_coord)

    def start(self):
        while not rospy.is_shutdown():

            map_list = [
                Point(0, 1),
                Point(0, 2),
                Point(1, 2),
                Point(2, 2),
                Point(2, 3),
                Point(2, 4),
                Point(3, 4),
                Point(4, 4),
                Point(4, 3),
                Point(4, 2),
                Point(3, 2),
                Point(2, 2),
                Point(2, 1)
            ]

            while map_list:
                next_coord = map_list.pop(0)
                self.move_to(next_coord)
                print(next_coord)
                self.current_point = next_coord
                sleep(15)



def main():
    path_finder = PathFinder()
    # sleep(2)
    path_finder.robot.r_init_node()
    sleep(5)
    path_finder.start()


if __name__ == '__main__':
    main()
