from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            # add here some image processing
            im_h = img.shape[0]
            im_w = img.shape[1]

            hsv_frame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_frame, (20, 100, 100), (30, 255, 255))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            if contours:
                for contour in contours:
                    (x_min, y_min, box_width, box_height) = cv2.boundingRect(contour)
                    x0, y0 = x_min - 15, y_min - 15
                    x1, y1 = x_min + box_width + 15, y_min + box_height + 15
                    point = (int(x0 + (x1 - x0) / 2), y1)
                    if 0.25 * im_w < point[0] < 0.75 * im_w and point[1] >= 0.75 * im_h:
                        condition = False

            env.render()
