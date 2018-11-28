#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
import subprocess, signal


class Window(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)

        self._layout = QtGui.QVBoxLayout(self)


        self._button_callbacks = [('Learn Contact Model', self.handleLearnContactModel),
                             ('Acquire Cloud', self.handleAcquireCloud),
                             ('Learn Contact Query', self.handleLearnContactQuery),
                             ('Generate Grasps', self.handleGenerateGrasps),
                             ('Next Grasp Solution', self.handleNextSolution),
                             ('Prev Grasp Solution', self.handlePrevSolution),
                             ('Execute Grasp', self.handleExecuteGrasp),
                             ('Restart World', self.handleRestartWorld),
                             ('Clear Solutions', self.handleClearSolutions),
                             ('Toggle Cloud Update', self.handleToggleCloudUpdate)
        ]

        self._buttons = []
        for name, handle in self._button_callbacks:
            button = QtGui.QPushButton(name, self)
            button.clicked.connect(handle)
            self._buttons.append(button)

            self._layout.addWidget(button)


        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)


    def handleExecuteGrasp(self):
        command = 'rosrun kinova_grasp moveit_grasp_commander.py'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output
    
    def handleRestartWorld(self):
        command = 'rosrun kinova_tabletop_gazebo model_spawner.py'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output

    def handleLearnContactModel(self):
        command = 'rosservice call /grasp_service/learn_contact_model'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output

    def handleAcquireCloud(self):
        command = 'rosservice call /grasp_service/acquire_cloud'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output


    def handleLearnContactQuery(self):
        command = 'rosservice call /grasp_service/learn_contact_query'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output

    
    def handleGenerateGrasps(self):
        command = 'rosservice call /grasp_service/generate_grasps'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output

    def handleNextSolution(self):
        command = 'rosservice call /grasp_service/next_solution'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output

    def handlePrevSolution(self):
        command = 'rosservice call /grasp_service/prev_solution'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

    def handleClearSolutions(self):
        command = 'rosservice call /grasp_service/clear_solutions'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

    def handleToggleCloudUpdate(self):
        command = 'rosservice call /grasp_service/toggle_cloud_update'
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        print output
if __name__ == '__main__':

    import sys
    app = QtGui.QApplication(sys.argv)
    window = Window()
    window.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())           