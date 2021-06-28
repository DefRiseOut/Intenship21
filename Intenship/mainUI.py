import functools
import sys
import threading
import time

from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QMainWindow, QErrorMessage, QMessageBox
from PyQt5.QtCore import QTimer, QVariantAnimation, QAbstractAnimation
from design import Ui_MainWindow
from robot import MyRobot
from read_config import read_config, update_config, read_speed


"""
I do not implement functionality for the last tab yet (Run), because,
I guess, there is no implementation at robot side too.
"""


class MainWindow(Ui_MainWindow):
    def __init__(self, form):
        self.setupUi(form)
        self.robot = None

        R, z, A, B, C, D, left, right, V, max_length, com = read_config()
        self.error_msg = "No error yet."
        self.wheel_radius_input.setValue(R)
        self.distance_btw_input.setValue(z)
        self.max_length_input.setValue(max_length)
        self.port_input.setText(com)

        self.ax_input.setValue(A[0])
        self.ay_input.setValue(A[1])
        self.bx_input.setValue(B[0])
        self.by_input.setValue(B[1])
        self.cx_input.setValue(C[0])
        self.cy_input.setValue(C[1])
        self.dx_input.setValue(D[0])
        self.dy_input.setValue(D[1])

        self.timer = QTimer()
        self.s = 0

        self.config_submit_btn.setStyleSheet('background-color:grey;')
        self.phase_submit_btn.setStyleSheet('background-color:grey;')
        self.calibr_btn.setStyleSheet('background-color:grey;')
        self.run_phases_btn.setStyleSheet('background-color:grey;')
        self.left_speed_btn.setStyleSheet('background-color:grey;')
        self.right_speed_btn.setStyleSheet('background-color:grey;')

        self.legs_left_input.setText(str(left))
        self.legs_right_input.setText(str(right))
        self.run = threading.Thread()
        self.add_events()

    def add_events(self):
        self.run_btn.clicked.connect(self.run_program)
        self.stop_btn.clicked.connect(self.stop_program)
        self.config_submit_btn.clicked.connect(self.update_config_values)
        self.phase_submit_btn.clicked.connect(self.set_phase)
        self.run_phases_btn.clicked.connect(self.run_phases)
        self.calibr_btn.clicked.connect(self.calibrate)
        self.save_btn.clicked.connect(self.save)
        self.load_btn.clicked.connect(self.load)
        self.right_speed_btn.clicked.connect(self.change_right_speed)
        self.left_speed_btn.clicked.connect(self.change_left_speed)
        self.timer.timeout.connect(self.LCDEvent)

    def LCDEvent(self):
        self.s += 1
        self.timer_lcd.display(self.s / 10)

    def change_right_speed(self):
        value = self.right_speed_input.value()
        self.robot.recalculate_speed_right(value)
        self.apply_color_animation(
            self.right_speed_btn,
            QColor("green"),
            QColor("grey"),
            duration=2000,
        )

    def change_left_speed(self):
        value = self.left_speed_input.value()
        self.robot.recalculate_speed_left(value)
        self.apply_color_animation(
            self.left_speed_btn,
            QColor("green"),
            QColor("grey"),
            duration=2000,
        )

    def save(self):
        self.robot.save_state()

    def load(self):
        if self.robot.stop == 1:
            self.robot.recover_state()

    def calibrate(self):
        if self.robot is not None:
            if self.robot.stop == 1:
                self.robot.clear_log()
                self.calibr_btn.setText("Calibrating...")
                self.statusbar.clearMessage()
                self.run = threading.Thread(target=self.robot.calibrate, args=(self.finish_calibrate, self.error))
                self.run.daemon = True
                self.run.start()

    def run_program(self):
        if self.robot.stop == 1:
            self.timer.start(100)
            self.run_btn.setText("Running...")
            self.robot.clear_log()
            self.statusbar.clearMessage()
            self.run = threading.Thread(target=self.robot.inf_loop, args=(self.finish_run, self.error))
            self.run.daemon = True
            self.run.start()

    def finish_setup_phase(self):
        self.apply_color_animation(
            self.phase_submit_btn,
            QColor("green"),
            QColor("grey"),
            duration=2000,
        )

    def finish_before_start(self):
        self.run_phases_btn.setText("Запустить все фазы")
        self.apply_color_animation(
            self.run_phases_btn,
            QColor("green"),
            QColor("grey"),
            duration=2000,
        )
        print("Before_start has finished.")

    def finish_run(self):
        self.run_btn.setText("Run")
        print("The main step cycle was finished.")

    def finish_calibrate(self):
        self.calibr_btn.setText("Calibrate")
        self.apply_color_animation(
            self.calibr_btn,
            QColor("green"),
            QColor("grey"),
            duration=2000,
        )
        print("Calibration has stopped")

    def stop_program(self):
        self.timer.stop()
        self.s = 0
        self.robot.force_stop()

    def set_phase(self):
        self.statusbar.clearMessage()
        is_right_checked = self.right_type_of_leg_radio.isChecked()
        number_of_leg = int(self.number_of_leg_input.text())
        number_of_phase = int(self.number_of_phase_input.text())
        location = self.location_input.value()
        self.robot.set_target_1_leg(is_right_checked, number_of_leg, number_of_phase, location, self.finish_setup_phase, self.error)

    def error(self, error_msg):
        self.statusbar.clearMessage()
        self.statusbar.showMessage(error_msg)

    def helper_function(self, widget, color):
        widget.setStyleSheet("background-color: {}".format(color.name()))

    def apply_color_animation(self, widget, start_color, end_color, duration=1000):
        anim = QVariantAnimation(
            widget,
            duration=duration,
            startValue=start_color,
            endValue=end_color,
            loopCount=1,
        )
        anim.valueChanged.connect(functools.partial(self.helper_function, widget))
        anim.start(QAbstractAnimation.DeleteWhenStopped)

    def run_phases(self):
        if self.robot.stop == 1:
            self.robot.clear_log()
            self.run_phases_btn.setText("Запускаю...")
            self.statusbar.clearMessage()
            self.run = threading.Thread(target=self.robot.before_start_init, args=(self.finish_before_start, self.error))
            self.run.daemon = True
            self.run.start()

    def update_config_values(self):
        self.config_submit_btn.setStyleSheet('background-color:red;')
        wheel_radius = self.wheel_radius_input.value()
        distance_btw = self.distance_btw_input.value()
        max_length = self.max_length_input.value()
        legs_right = eval(self.legs_right_input.text())
        legs_left = eval(self.legs_left_input.text())
        ax = self.ax_input.value()
        ay = self.ay_input.value()
        bx = self.bx_input.value()
        by = self.by_input.value()
        cx = self.cx_input.value()
        cy = self.cy_input.value()
        dx = self.dx_input.value()
        dy = self.dy_input.value()
        com = self.port_input.text()

        try:
            update_config("R", wheel_radius)
            update_config("z", distance_btw)
            update_config("max_length", max_length)
            update_config("A", [ax, ay])
            update_config("B", [bx, by])
            update_config("C", [cx, cy])
            update_config("D", [dx, dy])
            update_config("left", legs_left)
            update_config("right", legs_right)
            update_config("com", com)
            V = read_speed()
            self.robot = MyRobot(legs_left, legs_right, V, [[ax, ay], [bx, by], [cx, cy], [dx, dy]], distance_btw,
                                 wheel_radius, max_length, com)
            self.apply_color_animation(
                self.config_submit_btn,
                QColor("green"),
                QColor("grey"),
                duration=2000,
            )
        except Exception as e:
            print(e)
            self.apply_color_animation(
                self.config_submit_btn,
                QColor("red"),
                QColor("grey"),
                duration=2000,
            )


def main():
    app = QApplication(sys.argv)
    window = QMainWindow()
    ui = MainWindow(window)
    window.show()
    sys.exit(app.exec_())


uiThread = threading.Thread(target=main, args=())
uiThread.daemon = False
uiThread.start()
