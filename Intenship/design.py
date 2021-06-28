# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'design.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QIntValidator,QDoubleValidator,QFont

"""
Just design,
Do not edit this file unless you know what you are doing
"""

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(900, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 10, 621, 561))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_config = QtWidgets.QWidget()
        self.tab_config.setObjectName("tab_config")
        self.wheel_radius_label = QtWidgets.QLabel(self.tab_config)
        self.wheel_radius_label.setGeometry(QtCore.QRect(10, 20, 150, 20))
        self.wheel_radius_label.setObjectName("wheel_radius_label")
        self.wheel_radius_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.wheel_radius_input.setGeometry(QtCore.QRect(10, 50, 80, 31))
        self.wheel_radius_input.setObjectName("wheel_radius_input")
        self.distance_btw_label = QtWidgets.QLabel(self.tab_config)
        self.distance_btw_label.setGeometry(QtCore.QRect(10, 100, 181, 61))
        self.distance_btw_label.setWordWrap(True)
        self.distance_btw_label.setObjectName("distance_btw_label")
        self.distance_btw_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.distance_btw_input.setGeometry(QtCore.QRect(10, 160, 80, 31))
        self.distance_btw_input.setObjectName("distance_btw_input")
        self.max_length_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.max_length_input.setGeometry(QtCore.QRect(10, 280, 80, 31))
        self.max_length_input.setObjectName("max_length_input")
        self.max_length_label = QtWidgets.QLabel(self.tab_config)
        self.max_length_label.setGeometry(QtCore.QRect(10, 210, 181, 61))
        self.max_length_label.setWordWrap(True)
        self.max_length_label.setObjectName("max_length_label")

        self.port_input = QtWidgets.QLineEdit(self.tab_config)
        self.port_input.setGeometry(QtCore.QRect(10, 360, 80, 31))
        self.port_input.setObjectName("port_input")
        self.port_label = QtWidgets.QLabel(self.tab_config)
        self.port_label.setGeometry(QtCore.QRect(10, 310, 181, 61))
        self.port_label.setWordWrap(True)
        self.port_label.setObjectName("port_label")

        self.config_submit_btn = QtWidgets.QPushButton(self.tab_config)
        self.config_submit_btn.setGeometry(QtCore.QRect(40, 460, 110, 30))
        self.config_submit_btn.setObjectName("config_submit_btn")
        self.legs_id_left_label = QtWidgets.QLabel(self.tab_config)
        self.legs_id_left_label.setGeometry(QtCore.QRect(410, 20, 210, 50))
        self.legs_id_left_label.setWordWrap(True)
        self.legs_id_left_label.setObjectName("legs_id_left_label")
        self.legs_id_right_label = QtWidgets.QLabel(self.tab_config)
        self.legs_id_right_label.setGeometry(QtCore.QRect(190, 20, 210, 50))
        self.legs_id_right_label.setWordWrap(True)
        self.legs_id_right_label.setObjectName("legs_id_right_label")
        self.legs_left_input = QtWidgets.QLineEdit(self.tab_config)
        self.legs_left_input.setGeometry(QtCore.QRect(410, 80, 113, 30))
        self.legs_left_input.setObjectName("legs_left_input")
        self.legs_right_input = QtWidgets.QLineEdit(self.tab_config)
        self.legs_right_input.setGeometry(QtCore.QRect(190, 80, 113, 30))
        self.legs_right_input.setObjectName("legs_right_input")
        self.cy_label = QtWidgets.QLabel(self.tab_config)
        self.cy_label.setGeometry(QtCore.QRect(420, 340, 150, 20))
        self.cy_label.setObjectName("cy_label")
        self.ay_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.ay_input.setGeometry(QtCore.QRect(420, 230, 80, 31))
        self.ay_input.setObjectName("ay_input")
        self.dx_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.dx_input.setGeometry(QtCore.QRect(240, 440, 80, 31))
        self.dx_input.setObjectName("dx_input")
        self.cx_label = QtWidgets.QLabel(self.tab_config)
        self.cx_label.setGeometry(QtCore.QRect(240, 340, 150, 20))
        self.cx_label.setObjectName("cx_label")
        self.ax_label = QtWidgets.QLabel(self.tab_config)
        self.ax_label.setGeometry(QtCore.QRect(240, 200, 150, 20))
        self.ax_label.setObjectName("ax_label")
        self.ax_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.ax_input.setGeometry(QtCore.QRect(240, 230, 80, 31))
        self.ax_input.setObjectName("ax_input")
        self.dy_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.dy_input.setGeometry(QtCore.QRect(420, 440, 80, 31))
        self.dy_input.setObjectName("dy_input")
        self.ay_label = QtWidgets.QLabel(self.tab_config)
        self.ay_label.setGeometry(QtCore.QRect(420, 200, 150, 20))
        self.ay_label.setObjectName("ay_label")
        self.label = QtWidgets.QLabel(self.tab_config)
        self.label.setGeometry(QtCore.QRect(240, 140, 251, 51))
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.cy_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.cy_input.setGeometry(QtCore.QRect(420, 370, 80, 31))
        self.cy_input.setObjectName("cy_input")
        self.bx_label = QtWidgets.QLabel(self.tab_config)
        self.bx_label.setGeometry(QtCore.QRect(240, 270, 150, 20))
        self.bx_label.setObjectName("bx_label")
        self.by_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.by_input.setGeometry(QtCore.QRect(420, 300, 80, 31))
        self.by_input.setObjectName("by_input")
        self.by_label = QtWidgets.QLabel(self.tab_config)
        self.by_label.setGeometry(QtCore.QRect(420, 270, 150, 20))
        self.by_label.setObjectName("by_label")
        self.dy_label = QtWidgets.QLabel(self.tab_config)
        self.dy_label.setGeometry(QtCore.QRect(420, 410, 150, 20))
        self.dy_label.setObjectName("dy_label")
        self.dx_label = QtWidgets.QLabel(self.tab_config)
        self.dx_label.setGeometry(QtCore.QRect(240, 410, 150, 20))
        self.dx_label.setObjectName("dx_label")
        self.cx_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.cx_input.setGeometry(QtCore.QRect(240, 370, 80, 31))
        self.cx_input.setObjectName("cx_input")
        self.bx_input = QtWidgets.QDoubleSpinBox(self.tab_config)
        self.bx_input.setGeometry(QtCore.QRect(240, 300, 80, 31))
        self.bx_input.setObjectName("bx_input")
        self.tabWidget.addTab(self.tab_config, "")
        self.tab_before_start = QtWidgets.QWidget()
        self.tab_before_start.setObjectName("tab_before_start")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.tab_before_start)
        self.plainTextEdit.setGeometry(QtCore.QRect(10, 20, 601, 241))
        self.plainTextEdit.setReadOnly(True)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.left_type_of_leg_radio = QtWidgets.QRadioButton(self.tab_before_start)
        self.left_type_of_leg_radio.setGeometry(QtCore.QRect(20, 280, 135, 28))
        self.left_type_of_leg_radio.setObjectName("left_type_of_leg_radio")
        self.left_type_of_leg_radio.setChecked(True)
        self.right_type_of_leg_radio = QtWidgets.QRadioButton(self.tab_before_start)
        self.right_type_of_leg_radio.setGeometry(QtCore.QRect(20, 310, 135, 28))
        self.right_type_of_leg_radio.setObjectName("right_type_of_leg_radio")
        self.number_of_leg_label = QtWidgets.QLabel(self.tab_before_start)
        self.number_of_leg_label.setGeometry(QtCore.QRect(20, 350, 121, 22))
        self.number_of_leg_label.setObjectName("number_of_leg_label")
        self.number_of_leg_input = QtWidgets.QLineEdit(self.tab_before_start)
        self.number_of_leg_input.setGeometry(QtCore.QRect(20, 380, 80, 31))
        self.number_of_leg_input.setObjectName("number_of_leg_input")
        self.number_of_leg_input.setValidator(QIntValidator())
        self.number_of_phase_input = QtWidgets.QLineEdit(self.tab_before_start)
        self.number_of_phase_input.setGeometry(QtCore.QRect(190, 310, 80, 31))
        self.number_of_phase_input.setObjectName("number_of_phase_input")
        self.number_of_phase_input.setValidator(QIntValidator())
        self.number_of_phase_label = QtWidgets.QLabel(self.tab_before_start)
        self.number_of_phase_label.setGeometry(QtCore.QRect(190, 280, 121, 22))
        self.number_of_phase_label.setObjectName("number_of_phase_label")
        self.location_input = QtWidgets.QDoubleSpinBox(self.tab_before_start)
        self.location_input.setMaximum(1)
        self.location_input.setMinimum(0)
        self.location_input.setGeometry(QtCore.QRect(190, 380, 80, 31))
        self.location_input.setObjectName("location_input")
        self.location_label = QtWidgets.QLabel(self.tab_before_start)
        self.location_label.setGeometry(QtCore.QRect(190, 350, 191, 22))
        self.location_label.setObjectName("location_label")
        self.phase_submit_btn = QtWidgets.QPushButton(self.tab_before_start)
        self.phase_submit_btn.setGeometry(QtCore.QRect(90, 450, 110, 30))
        self.phase_submit_btn.setObjectName("phase_submit_btn")
        self.run_phases_btn = QtWidgets.QPushButton(self.tab_before_start)
        self.run_phases_btn.setGeometry(QtCore.QRect(410, 330, 191, 110))
        self.run_phases_btn.setObjectName("run_phases_btn")
        self.tabWidget.addTab(self.tab_before_start, "")
        self.tab_run = QtWidgets.QWidget()
        self.tab_run.setObjectName("tab_run")
        self.right_speed_input = QtWidgets.QDoubleSpinBox(self.tab_run)
        self.right_speed_input.setGeometry(QtCore.QRect(200, 70, 80, 31))
        self.right_speed_input.setObjectName("right_speed_input")
        self.right_speed_input.setMaximum(0.03)
        self.right_speed_input.setMinimum(-0.03)
        self.left_speed_input = QtWidgets.QDoubleSpinBox(self.tab_run)
        self.left_speed_input.setGeometry(QtCore.QRect(20, 70, 80, 31))
        self.left_speed_input.setObjectName("left_speed_input")
        self.left_speed_input.setMaximum(0.03)
        self.left_speed_input.setMinimum(-0.03)
        self.left_speed_label = QtWidgets.QLabel(self.tab_run)
        self.left_speed_label.setGeometry(QtCore.QRect(20, 20, 200, 50))
        self.left_speed_label.setWordWrap(True)
        self.left_speed_label.setObjectName("left_speed_label")
        self.right_speed_label = QtWidgets.QLabel(self.tab_run)
        self.right_speed_label.setGeometry(QtCore.QRect(200, 20, 200, 50))
        self.right_speed_label.setWordWrap(True)
        self.right_speed_label.setObjectName("right_speed_label")
        self.left_speed_btn = QtWidgets.QPushButton(self.tab_run)
        self.left_speed_btn.setGeometry(QtCore.QRect(20, 110, 80, 30))
        self.left_speed_btn.setObjectName("left_speed_btn")
        self.right_speed_btn = QtWidgets.QPushButton(self.tab_run)
        self.right_speed_btn.setGeometry(QtCore.QRect(200, 110, 80, 30))
        self.right_speed_btn.setStyleSheet("")
        self.right_speed_btn.setObjectName("right_speed_btn")
        self.tabWidget.addTab(self.tab_run, "")
        self.timer_lcd = QtWidgets.QLCDNumber(self.centralwidget)
        self.timer_lcd.setGeometry(QtCore.QRect(750, 40, 100, 30))
        self.timer_lcd.setObjectName("timer_lcd")
        self.time_label = QtWidgets.QLabel(self.centralwidget)
        self.time_label.setGeometry(QtCore.QRect(650, 40, 100, 30))
        self.time_label.setObjectName("time_label")
        self.lcdNumber_2 = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber_2.setGeometry(QtCore.QRect(750, 90, 100, 30))
        self.lcdNumber_2.setObjectName("lcdNumber_2")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(650, 90, 100, 30))
        self.label_2.setObjectName("label_2")
        self.run_btn = QtWidgets.QPushButton(self.centralwidget)
        self.run_btn.setGeometry(QtCore.QRect(650, 530, 110, 30))
        self.run_btn.setObjectName("run_btn")
        self.stop_btn = QtWidgets.QPushButton(self.centralwidget)
        self.stop_btn.setGeometry(QtCore.QRect(770, 530, 110, 30))
        self.stop_btn.setObjectName("stop_btn")

        self.save_btn = QtWidgets.QPushButton(self.centralwidget)
        self.save_btn.setGeometry(QtCore.QRect(650, 410, 110, 30))
        self.save_btn.setObjectName("save_btn")
        self.load_btn = QtWidgets.QPushButton(self.centralwidget)
        self.load_btn.setGeometry(QtCore.QRect(770, 410, 110, 30))
        self.load_btn.setObjectName("load_btn")

        self.calibr_btn = QtWidgets.QPushButton(self.centralwidget)
        self.calibr_btn.setGeometry(QtCore.QRect(650, 470, 110, 30))
        self.calibr_btn.setObjectName("calibr_btn")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.wheel_radius_label.setText(_translate("MainWindow", "Радиус колеса, м"))
        self.distance_btw_label.setText(_translate("MainWindow", "Расстояние между моторами, м"))
        self.max_length_label.setText(_translate("MainWindow", "Максимальная длина раскрутки нити, м"))
        self.port_label.setText(_translate("MainWindow", "Название порта"))
        self.config_submit_btn.setText(_translate("MainWindow", "OK"))
        self.legs_id_left_label.setText(_translate("MainWindow", "Введите ID левых ног  в формате [[1,2],[3,4]]"))
        self.legs_id_right_label.setText(_translate("MainWindow", "Введите ID правых ног в формате [[1,2],[3,4]]"))
        self.cy_label.setText(_translate("MainWindow", "C.y"))
        self.cx_label.setText(_translate("MainWindow", "C.x"))
        self.ax_label.setText(_translate("MainWindow", "A.x"))
        self.ay_label.setText(_translate("MainWindow", "A.y"))
        self.label.setText(_translate("MainWindow", "Координаты изначальной трапеции. Точки A, B, C, D "))
        self.bx_label.setText(_translate("MainWindow", "B.x"))
        self.by_label.setText(_translate("MainWindow", "B.y"))
        self.dy_label.setText(_translate("MainWindow", "D.y"))
        self.dx_label.setText(_translate("MainWindow", "D.x"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_config), _translate("MainWindow", "Config"))
        self.plainTextEdit.setPlainText(_translate("MainWindow", "Перестройка ноги в определенное положение.\n"
"1) Выберите тип ноги: левая или правая\n"
"2) Напишите номер ноги\n"
"3) Выберите фазу (число)\n"
"AB - 1\n"
"BC - 2\n"
"CD - 3\n"
"DA - 4\n"
"4) Напишите процентное соотношение\n"
"5) Нажмите ОК"))
        self.left_type_of_leg_radio.setText(_translate("MainWindow", "Левая нога"))
        self.right_type_of_leg_radio.setText(_translate("MainWindow", "Правая нога"))
        self.number_of_leg_label.setText(_translate("MainWindow", "Номер ноги"))
        self.number_of_phase_label.setText(_translate("MainWindow", "Номер фазы"))
        self.location_label.setText(_translate("MainWindow", "Местоположение"))
        self.phase_submit_btn.setText(_translate("MainWindow", "ОК"))
        self.run_phases_btn.setText(_translate("MainWindow", "Запустить все фазы"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_before_start), _translate("MainWindow", "Before start"))
        self.left_speed_label.setText(_translate("MainWindow", "Скорость левого борта, м/с"))
        self.right_speed_label.setText(_translate("MainWindow", "Скорость правого борта, м/с"))
        self.left_speed_btn.setText(_translate("MainWindow", "OK"))
        self.right_speed_btn.setText(_translate("MainWindow", "OK"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_run), _translate("MainWindow", "Run"))
        self.time_label.setText(_translate("MainWindow", "Time:"))
        self.label_2.setText(_translate("MainWindow", "Ток"))
        self.run_btn.setText(_translate("MainWindow", "Run"))
        self.stop_btn.setText(_translate("MainWindow", "Stop"))
        self.calibr_btn.setText(_translate("MainWidow", "Calibrate"))
        self.load_btn.setText(_translate("MainWidow", "Load"))
        self.save_btn.setText(_translate("MainWidow", "Save"))
