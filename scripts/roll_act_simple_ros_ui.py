# Create a PYQT5 UI that does:
# ROS subs for /bigss_maxon_can_ros_node/measured_js (sensor_msgs/JointState)
# ROS pubs  for /bigss_maxon_can_ros_node/move_jp (sensor_msgs/JointState) /bigss_maxon_can_ros_node/servo_jv (sensor_msgs/JointState)
# calls these services: /bigss_maxon_can_ros_node/disable /bigss_maxon_can_ros_node/enable /bigss_maxon_can_ros_node/set_cmd_mode_pos /bigss_maxon_can_ros_node/set_cmd_mode_vel (all are std_srvs/Trigger)
# for each service call make a button, for the subs make a label, for the pubs make a settable value and a button to send the value

import sys
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore
import rospy
import signal

color_button_activated = 'background-color: rgba(30, 25, 141, 0.95)'
color_button_deactivated = 'background-color: rgba(58, 47, 214, 0.45)'
color_positive_indicator = 'background-color: rgba(39, 245, 157, 0.8)'
color_negative_indicator = 'background-color: rgba(226, 39, 29, 0.48)'

class FloatDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, decimals, parent=None):
        super(FloatDelegate, self).__init__(parent=parent)
        self.nDecimals = decimals

    def displayText(self, value, locale):
        try:
            number = float(value)
        except ValueError:
            return super(FloatDelegate, self).displayText(value, locale)
        else:
            return "%0.3f" % number

class SimpleRosUi(QtWidgets.QWidget):
    def __init__(self, namespace, window_title="UI"):
        super(SimpleRosUi, self).__init__()
        self.system_namespace = namespace
        self.window = QtWidgets.QMainWindow()
        self.initUI(window_title)
        self.initRos()
        self.initWidgets()
        self.initTimer()
        self.initLayout()
        self.show()
        self.update()
        self.show()
        self.raise_()
        self.activateWindow()
        self.showNormal()


    def initUI(self, window_title):
        self.setWindowTitle(window_title)
        # self.setWindowIcon(QtGui.QIcon('icon.png'))
        screen_resolution = QtWidgets.QDesktopWidget().screenGeometry()
        self.screen_width, self.screen_height = screen_resolution.width(), screen_resolution.height()
        self.setGeometry(0, 0, int(self.screen_width/2), self.screen_height)
        display_monitor = 0 # the number of the monitor you want to display your widget
        monitor = QtWidgets.QDesktopWidget().screenGeometry(display_monitor)
        self.move(monitor.left(), monitor.top())
        self.show()

    def initRos(self):
        rospy.init_node('simple_ros_ui', anonymous=True)

        self.measured_jv = 0.0
        self.measured_jp = 0.0
        self.measured_js = JointState()
        self.measured_js_sub = rospy.Subscriber(self.system_namespace+'measured_js', JointState, self.measured_js_callback)
        self.is_homed = False
        self.is_homed_sub = rospy.Subscriber(self.system_namespace+'is_homed', Bool, self.is_homed_callback)
        self.is_enabled = False
        self.is_enabled_sub = rospy.Subscriber(self.system_namespace+'is_enabled', Bool, self.is_enabled_callback)


        self.move_jp_pub = rospy.Publisher(self.system_namespace+'move_jp', JointState, queue_size=1)
        self.send_servo_jv_cmd = False
        self.servo_jv_pub = rospy.Publisher(self.system_namespace+'servo_jv', JointState, queue_size=1)

        self.disable_srv = rospy.ServiceProxy(self.system_namespace+'disable', Trigger)
        self.enable_srv = rospy.ServiceProxy(self.system_namespace+'enable', Trigger)
        self.set_cmd_mode_pos_srv = rospy.ServiceProxy(self.system_namespace+'set_cmd_mode_pos', Trigger)
        self.set_cmd_mode_vel_srv = rospy.ServiceProxy(self.system_namespace+'set_cmd_mode_vel', Trigger)
        self.home_srv = rospy.ServiceProxy(self.system_namespace+'home', Trigger)
        self.halt_srv = rospy.ServiceProxy(self.system_namespace+'halt', Trigger)

    def initWidgets(self):
        # position and velocity current values
        self.measured_js_label = QtWidgets.QLabel('Measured Joint State')
        self.measured_js_label.setAlignment(QtCore.Qt.AlignCenter)
        self.measured_js_label.setStyleSheet('font: 12pt')
        self.measured_js_table = QtWidgets.QTableWidget()
        self.measured_js_table.setColumnCount(2)
        self.measured_js_table.setHorizontalHeaderLabels(['Position', 'Velocity'])
        self.measured_js_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.measured_js_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.measured_js_table.setItemDelegate(FloatDelegate(3))
        self.measured_js_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.measured_js_table.setRowCount(1)

        #buttons for services
        self.disable_button = QtWidgets.QPushButton('Disable')
        self.disable_button.setStyleSheet(color_button_deactivated)
        self.disable_button.clicked.connect(self.disable_srv_clicked)
        self.enable_button = QtWidgets.QPushButton('Enable')
        self.enable_button.setStyleSheet(color_button_deactivated)
        self.enable_button.clicked.connect(self.enable_srv_clicked)
        self.set_cmd_mode_pos_button = QtWidgets.QPushButton('Set Position Mode')
        self.set_cmd_mode_pos_button.setStyleSheet(color_button_deactivated)
        self.set_cmd_mode_pos_button.clicked.connect(self.set_cmd_mode_pos_clicked)
        self.set_cmd_mode_vel_button = QtWidgets.QPushButton('Set Velocity Mode')
        self.set_cmd_mode_vel_button.setStyleSheet(color_button_deactivated)
        self.set_cmd_mode_vel_button.clicked.connect(self.set_cmd_mode_vel_clicked)
        self.home_button = QtWidgets.QPushButton('Home')
        self.home_button.setStyleSheet(color_button_deactivated)
        self.home_button.clicked.connect(self.home_srv_clicked)
        self.halt_button = QtWidgets.QPushButton('Halt')
        self.halt_button.setStyleSheet(color_button_deactivated)
        self.halt_button.clicked.connect(self.halt_srv_clicked)
        
        # manual position and velocity commands
        self.move_jp_label = QtWidgets.QLabel('Manual Joint Position Command')
        self.move_jp_label.setAlignment(QtCore.Qt.AlignCenter)
        self.move_jp_label.setStyleSheet('font: 12pt')
        self.move_jp_table = QtWidgets.QTableWidget()
        self.move_jp_table.setColumnCount(2)
        self.move_jp_table.setHorizontalHeaderLabels(['Position', 'Velocity'])
        self.move_jp_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.move_jp_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.move_jp_table.setItemDelegate(FloatDelegate(3))
        self.move_jp_table.setRowCount(1)
        self.move_jp_table.setItem(0, 0, QtWidgets.QTableWidgetItem(str(0.0)))
        self.move_jp_table.setItem(0, 1, QtWidgets.QTableWidgetItem(str(0.0)))

        self.servo_jv_label = QtWidgets.QLabel('Manual Joint Velocity Command')
        self.servo_jv_label.setAlignment(QtCore.Qt.AlignCenter)
        self.servo_jv_label.setStyleSheet('font: 12pt')
        self.servo_jv_table = QtWidgets.QTableWidget()
        self.servo_jv_table.setColumnCount(1)
        self.servo_jv_table.setHorizontalHeaderLabels(['Velocity'])
        self.servo_jv_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.servo_jv_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.servo_jv_table.setItemDelegate(FloatDelegate(3))
        self.servo_jv_table.setRowCount(1)
        self.servo_jv_table.setItem(0, 0, QtWidgets.QTableWidgetItem(str(0.0)))

        # button to send manual position and velocity commands
        self.send_pos_cmd_button = QtWidgets.QPushButton('Send Position Command')
        self.send_pos_cmd_button.setStyleSheet(color_button_deactivated)
        self.send_pos_cmd_button.clicked.connect(self.send_pos_cmd_clicked)

        self.send_vel_cmd_button = QtWidgets.QPushButton('Send Velocity Command')
        self.send_vel_cmd_button.setStyleSheet(color_button_deactivated)
        self.send_vel_cmd_button.clicked.connect(self.send_vel_cmd_clicked)

        self.stop_vel_cmd_button = QtWidgets.QPushButton('Stop Velocity Command')
        self.stop_vel_cmd_button.setStyleSheet(color_button_deactivated)
        self.stop_vel_cmd_button.clicked.connect(self.stop_vel_cmd_clicked)

        # indicators for homing and enabling whose color changes depending on the state, not buttons but labels with background color
        self.is_homed_indicator = QtWidgets.QLabel('Homed')
        self.is_homed_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.is_homed_indicator.setStyleSheet('font: 12pt')
        self.is_homed_indicator.setStyleSheet(color_negative_indicator)

        self.is_enabled_indicator = QtWidgets.QLabel('Enabled')
        self.is_enabled_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.is_enabled_indicator.setStyleSheet('font: 12pt')
        self.is_enabled_indicator.setStyleSheet(color_negative_indicator)

    def initTimer(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def initLayout(self):
        self.main_layout = QtWidgets.QVBoxLayout()

        self.disclaimer_label = QtWidgets.QLabel('This is a simple UI to interact with BIGSSMaxonCANROS for simple testing and debugging. Please feel free to modify it to your needs.')
        self.disclaimer_label.setWordWrap(True)
        self.disclaimer_label.setAlignment(QtCore.Qt.AlignCenter)
        self.disclaimer_label.setStyleSheet('font: 12pt')
        self.main_layout.addWidget(self.disclaimer_label)
        
        self.state_data_layout = QtWidgets.QHBoxLayout()
        self.state_data_layout.addWidget(self.measured_js_label)
        self.state_data_layout.addWidget(self.measured_js_table)
        self.state_data_layout.addWidget(self.is_homed_indicator)
        self.state_data_layout.addWidget(self.is_enabled_indicator)
        self.main_layout.addLayout(self.state_data_layout)

        self.button_layout = QtWidgets.QHBoxLayout()
        self.button_layout.addWidget(self.disable_button)
        self.button_layout.addWidget(self.enable_button)
        self.button_layout.addWidget(self.set_cmd_mode_pos_button)
        self.button_layout.addWidget(self.set_cmd_mode_vel_button)
        self.button_layout.addWidget(self.home_button)
        self.button_layout.addWidget(self.halt_button)
        self.main_layout.addLayout(self.button_layout)
        
        # add a widget with tabs for position and velocity commands
        tab1 = QtWidgets.QWidget()
        tab1_layout = QtWidgets.QVBoxLayout()
        tab1_layout.addWidget(self.move_jp_label)
        tab1_layout.addWidget(self.move_jp_table)
        tab1_layout.addWidget(self.send_pos_cmd_button)
        tab1.setLayout(tab1_layout)

        tab2 = QtWidgets.QWidget()
        tab2_layout = QtWidgets.QVBoxLayout()
        tab2_layout.addWidget(self.servo_jv_label)
        tab2_layout.addWidget(self.servo_jv_table)
        tab2_layout.addWidget(self.send_vel_cmd_button)
        tab2_layout.addWidget(self.stop_vel_cmd_button)
        tab2.setLayout(tab2_layout)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(tab1, "Manual Position Command")
        tabs.addTab(tab2, "Manual Velocity Command")
        self.main_layout.addWidget(tabs)

        # add two tabs, one for position and one for velocity        

        # self.main_layout.addWidget(self.move_jp_label)
        # self.main_layout.addWidget(self.move_jp_table)
        # self.main_layout.addWidget(self.send_pos_cmd_button)
        # self.main_layout.addWidget(self.servo_jv_label)
        # self.main_layout.addWidget(self.servo_jv_table)
        # self.main_layout.addWidget(self.send_vel_cmd_button)
        # self.main_layout.addWidget(self.stop_vel_cmd_button)
        self.setLayout(self.main_layout)

    def measured_js_callback(self, msg):
        self.measured_jp = msg.position[0]
        self.measured_jv = msg.velocity[0]

    def is_homed_callback(self, msg):
        self.is_homed = msg.data

        
    def is_enabled_callback(self, msg):
        self.is_enabled = msg.data

    
    def disable_srv_clicked(self):
        self.disable_srv()

    def enable_srv_clicked(self):
        self.enable_srv()

    def set_cmd_mode_pos_clicked(self):
        self.set_cmd_mode_pos_srv()

    def set_cmd_mode_vel_clicked(self):
        self.set_cmd_mode_vel_srv()

    def home_srv_clicked(self):
        self.home_srv()

    def halt_srv_clicked(self):
        if self.send_servo_jv_cmd:
            self.send_servo_jv_cmd = False
            self.servo_jv = JointState()
            self.servo_jv.velocity = [0.0]
            self.servo_jv_pub.publish(self.servo_jv)
        self.halt_srv()

    def send_pos_cmd_clicked(self):
        self.move_jp = JointState()
        self.move_jp.position = [float(self.move_jp_table.item(0,0).text())]
        self.move_jp.velocity = [float(self.move_jp_table.item(0,1).text())]
        self.move_jp_pub.publish(self.move_jp)

    def send_vel_cmd_clicked(self):
        self.send_servo_jv_cmd = True
        self.active_servo_jv_cmd = self.servo_jv = JointState()
        self.servo_jv.velocity = [float(self.servo_jv_table.item(0,0).text())]

    def stop_vel_cmd_clicked(self):
        self.send_servo_jv_cmd = False
        self.servo_jv = JointState()
        self.servo_jv.velocity = [0.0]
        self.servo_jv_pub.publish(self.servo_jv)

    def update(self):
        self.measured_js_table.setRowCount(1)
        self.measured_js_table.setItem(0, 0, QtWidgets.QTableWidgetItem(str(self.measured_jp)))
        self.measured_js_table.setItem(0, 1, QtWidgets.QTableWidgetItem(str(self.measured_jv)))
        self.measured_js_table.resizeColumnsToContents()
        self.measured_js_table.resizeRowsToContents()
        self.is_homed_indicator.setStyleSheet(color_positive_indicator if self.is_homed else color_negative_indicator)
        self.is_enabled_indicator.setStyleSheet(color_positive_indicator if self.is_enabled else color_negative_indicator)

        if self.send_servo_jv_cmd:
            self.servo_jv_pub.publish(self.servo_jv)

    def closeEvent(self, event):
        self.timer.stop()
        self.disable_srv()
        event.accept()

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    QtWidgets.QApplication.quit()

def main():
    namespace = "/bigss_maxon_can_ros_node/"

    signal.signal(signal.SIGINT, sigint_handler) # https://stackoverflow.com/questions/4938723/what-is-the-correct-way-to-make-my-pyqt-application-quit-when-killed-from-the-co
    app = QtWidgets.QApplication(sys.argv)

    # for sigint handling (makes ctrl+c work)
    timer = QtCore.QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
   
    ui = SimpleRosUi(namespace)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()










