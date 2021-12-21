from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


def change_line_left(env):
    for i in range(0, 22):
        img, reward, done, info = env.step([0.5, 0.5])
        env.render()
    for i in range(0, 22):
        img, reward, done, info = env.step([0.5, -0.5])
        env.render()


def change_line_right(env):
    for i in range(0, 22):
        img, reward, done, info = env.step([0.5, -0.5])
        env.render()
    for i in range(0, 22):
        img, reward, done, info = env.step([0.5, 0.5])
        env.render()


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        condition = True
        lane_change_flag = False

        while condition:
            img, reward, done, info = env.step([1, 0])
            im_h = img.shape[0]
            im_w = img.shape[1]

            # img in RGB
            # add here some image processing
            hsv_frame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_frame, (20, 100, 100), (30, 255, 255))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            if contours:
                for contour in contours:
                    (x_min, y_min, box_width, box_height) = cv2.boundingRect(contour)
                    tl, br = (x_min - 15, y_min - 15), (x_min + box_width + 15, y_min + box_height + 15)
                    point = list(map(int, [tl[0] + (br[0] - tl[0]) / 2, br[1]]))

                    if 0.3 * im_w < point[0] < 0.7 * im_w and point[1] >= 0.5 * im_h:
                        lane_change_flag = True

            if lane_change_flag:
                change_line_left(env)
                change_line_right(env)

            lane_change_flag = False
            # condition = True
            env.render()
