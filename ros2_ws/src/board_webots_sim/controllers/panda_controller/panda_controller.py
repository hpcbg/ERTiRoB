from controller import Robot
import sys
import threading
import csv
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton, QLineEdit

initial = [0, 0, 0, -1.770800, -1.6, 1.6, 0.79, 0.04]

limits = [[-2.9671, 2.9671], [-1.8326, 1.8326], [-2.9671, 2.9671], [-3.1416, -0.4],
          [-2.9671, 2.9671], [-0.0873, 3.8223], [-2.9671, 2.9671], [0, 0.04]]


class PandaController(Robot):
  def __init__(self):
    super().__init__()
    self.time_step = int(self.getBasicTimeStep())

    self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
                        'panda_finger::right']
    self.joints = {name: self.getDevice(name) for name in self.joint_names}

    for i in range(8):
      self.joints[self.joint_names[i]].setPosition(initial[i])

  def set_joint_positions(self, positions):
    print(positions)
    for i, pos in enumerate(positions):
      self.joints[self.joint_names[i]].setPosition(pos)

  def step_robot(self):
    while self.step(self.time_step) != -1:
      pass


class PandaUI(QWidget):
  def __init__(self, controller):
    super().__init__()
    self.controller = controller
    self.movement_data = []
    self.init_ui()

  def init_ui(self):
    layout = QVBoxLayout()

    self.sliders = []
    self.labels = []
    self.text_boxes = []
    for i in range(8):
      h_layout = QHBoxLayout()
      label = QLabel(f'Joint {i+1}: {initial[i]:2.3f} rad')
      slider = QSlider()
      slider.setOrientation(1)
      slider.setMinimum(int(limits[i][0]*1000))
      slider.setMaximum(int(limits[i][1]*1000))
      slider.setValue(int(initial[i]*1000))
      slider.setTickInterval(10)
      slider.setSingleStep(1)
      slider.setFixedWidth(300)
      slider.valueChanged.connect(
          lambda value, i=i: self.update_robot(value, i))

      decrease_button = QPushButton("-")
      decrease_button.clicked.connect(
          lambda _, i=i: self.change_slider_value(i, -1))
      increase_button = QPushButton("+")
      increase_button.clicked.connect(
          lambda _, i=i: self.change_slider_value(i, 1))

      text_box = QLineEdit()
      text_box.setFixedWidth(60)
      text_box.setText(f'{initial[i]:2.3f}')
      text_box.returnPressed.connect(lambda i=i: self.update_from_text_box(i))

      h_layout.addWidget(label)
      h_layout.addWidget(decrease_button)
      h_layout.addWidget(slider)
      h_layout.addWidget(increase_button)
      h_layout.addWidget(text_box)
      layout.addLayout(h_layout)

      self.sliders.append(slider)
      self.labels.append(label)
      self.text_boxes.append(text_box)

    self.filename_input = QLineEdit()
    self.filename_input.setPlaceholderText('Enter filename')
    layout.addWidget(self.filename_input)

    self.point_count_label = QLabel('Number of points: 0')
    layout.addWidget(self.point_count_label)

    self.save_button = QPushButton('Save movement')
    self.save_button.clicked.connect(self.save_movement)
    layout.addWidget(self.save_button)

    self.clear_button = QPushButton('Clear movement')
    self.clear_button.clicked.connect(self.clear_movement)
    layout.addWidget(self.clear_button)

    self.setLayout(layout)
    self.setWindowTitle('Panda Controller')
    self.show()

  def update_robot(self, value, joint_index):
    rad_value = value / 1000
    self.labels[joint_index].setText(
        f'Joint {joint_index+1}: {rad_value:.3f} rad')
    self.text_boxes[joint_index].setText(f"{rad_value:.3f}")
    positions = [slider.value() / 1000 for slider in self.sliders]
    self.controller.set_joint_positions(positions)
    self.movement_data.append(positions[:])
    self.point_count_label.setText(
        f'Number of points: {len(self.movement_data)}')

  def update_from_text_box(self, index):
    try:
      value = float(self.text_boxes[index].text())
      slider_value = int(value * 1000)
      self.sliders[index].setValue(slider_value)
      self.update_robot(slider_value, index)
    except ValueError:
      self.text_boxes[index].setText('0.000')

  def change_slider_value(self, index, delta):
    new_value = self.sliders[index].value() + delta
    self.sliders[index].setValue(new_value)
    self.update_robot(new_value, index)

  def save_movement(self):
    filename = self.filename_input.text().strip()
    if not filename:
      filename = 'movement_data.csv'
    if not filename.endswith('.csv'):
      filename += '.csv'

    with open(filename, 'w', newline="") as file:
      writer = csv.writer(file)
      writer.writerow([f'Joint {i+1}' for i in range(7)])
      writer.writerows(self.movement_data)
    print(f"Movement is recorded in {filename}")

  def clear_movement(self):
    self.movement_data = []
    self.point_count_label.setText('Number of points: 0')
    print('Record is cleared.')


def run_ui(robot):
  app = QApplication(sys.argv)
  ui = PandaUI(robot)
  app.exec_()


if __name__ == "__main__":
  robot = PandaController()

  ui_thread = threading.Thread(target=run_ui, args=(robot,), daemon=True)
  ui_thread.start()

  robot.step_robot()
