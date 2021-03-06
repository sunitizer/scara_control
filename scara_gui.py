from tkinter import *
import tkinter.ttk as ttk
import serial
import serial.tools.list_ports
import math
import struct
import ScaraMotors
import numpy as np


def main():
    window = Tk()
    window.title("SCARA GUI")
    # window.geometry("350x250")
    angle_in = [0.0, 0.0]
    scaramotor1 = ScaraMotors.ScaraMotors(arm_len1=179.9, arm_len2=150, angle_res1=2 * math.pi / 200 / 3.70588235 / 8,
                                          angle_res2=2 * math.pi / 400 / 2 / 8, angle_init=angle_in)
    serial1 = SerialConnection(window)
    pointtopoint = PointtoPoint(window, scaramotor1, serial1)
    straight_line = LineInput(window, scaramotor1, serial1)
    window.mainloop()



class SerialConnection:

    def __init__(self, master):

        self.arduinosrl = serial.Serial()
        self.ports = serial.tools.list_ports.comports()
        self.portlist = []
        for self.p in self.ports:
            print(self.p.device)
            self.portlist.append(str(self.p.device))

        serframe = Frame(master)
        serframe.grid(column=0, row=0)

        self.lbl_srl = Label(serframe, text="Serial Port:", font=("Arial", 8))
        self.lbl_srl.grid(column=0, row=0)

        self.combo = ttk.Combobox(serframe, values=self.portlist)
        self.combo.current(0)
        self.combo.grid(column=1, row=0)

        self.btn_connect = Button(serframe, text="Connect", command=self.clickedConnectSrl, font=("Arial", 8))
        self.btn_connect.grid(column=2, row=0)

        self.lbl_PortConnect = Label(serframe, font=("Arial", 8))
        self.lbl_PortConnect.grid(columnspan=3)

    def clickedConnectSrl(self):
        if self.btn_connect['text'] == "Connect":
            self.arduinosrl.port = self.combo.get()
            self.arduinosrl.baudrate = 9600
            self.arduinosrl.open()
            self.lbl_PortConnect.configure(text="Connected to " + self.combo.get())
            self.btn_connect.configure(text="Disconnect")
        else:
            self.arduinosrl.close()
            self.btn_connect.configure(text="Connect")
            self.lbl_PortConnect.configure(text="Disconnected")

    def send_floats_serial(self, float_num):
        serial_float = struct.pack('ffff', float_num[0], float_num[1], float_num[2], float_num[3])
        try:
            self.arduinosrl.write(serial_float)
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))

    def serial_send_steps_dir_woffset(self, steps, direc):
        offset_array = [0, 0, 0, 0]
        motor1_msb, motor1_lsb = self.convert_to_bytes(steps[0])
        motor2_msb, motor2_lsb = self.convert_to_bytes(steps[1])
        motor1_msb, offset_array[0] = self.is_xff(motor1_msb)
        motor1_lsb, offset_array[1] = self.is_xff(motor1_lsb)
        motor2_msb, offset_array[2] = self.is_xff(motor2_msb)
        motor2_lsb, offset_array[3] = self.is_xff(motor2_lsb)
        off_byte = self.make_offset_byte(offset_array)
        dir_byte = self.make_offset_byte(direc)

        # send 7 bytes to ardiuno: 255, direction_byte, offset_byte, motor1_msb, motor1_lsb, motor2_msb, motor2_lsb
        #self.send_start_byte()
        self.arduinosrl.write(dir_byte)
        self.arduinosrl.write(off_byte)
        self.arduinosrl.write(motor1_msb)
        self.arduinosrl.write(motor1_lsb)
        self.arduinosrl.write(motor2_msb)
        self.arduinosrl.write(motor2_lsb)

    def serial_send_steps_dir(self, steps, direc):
        motor1_msb, motor1_lsb = self.convert_to_bytes(steps[0])
        motor2_msb, motor2_lsb = self.convert_to_bytes(steps[1])
        dir_byte = self.make_offset_byte(direc)

        # send 5 bytes to ardiuno:  direction_byte, motor1_msb, motor1_lsb, motor2_msb, motor2_lsb
        self.arduinosrl.write(dir_byte)
        self.arduinosrl.write(motor1_msb)
        self.arduinosrl.write(motor1_lsb)
        self.arduinosrl.write(motor2_msb)
        self.arduinosrl.write(motor2_lsb)

    def serial_send_two_int(self, array):
        int1_msb, int1_lsb = self.convert_to_bytes(array[0])
        int2_msb, int2_lsb = self.convert_to_bytes(array[1])

        # send 4 bytes to ardiuno:  int1_msb, int1_lsb, int2_msb, int2_lsb
        self.arduinosrl.write(int1_msb)
        self.arduinosrl.write(int1_lsb)
        self.arduinosrl.write(int2_msb)
        self.arduinosrl.write(int2_lsb)

    def send_start_byte(self):
        self.arduinosrl.write(struct.pack('>B', 0b11111111))

    def send_start_byte_line(self):
        self.arduinosrl.write(struct.pack('>B', 0b11111110))

    def convert_to_bytes(self, number):
        msb = struct.pack('>B', number >> 8)
        lsb = struct.pack('>B', number & 0b11111111)

        return msb, lsb

    def is_xff(self, byte):
        offset = 0
        if byte == struct.pack('>B', 0b11111111):
            byte = struct.pack('>B', 0b11111110)
            offset = 1
        return byte, offset

    def make_offset_byte(self, offset_array):
        byte = 0
        for ind in range(len(offset_array)):
            if offset_array[ind] == 1:
                byte = (1 | byte)
            if ind != len(offset_array)-1:
                byte = byte << 1
        return struct.pack('>B', byte)


class PointtoPoint:

    def __init__(self, master, scaramotor, serial1):

        frame_coord = Frame(master)
        frame_coord.grid(column=0, row=1)

        self.lbl_x_coord = Label(frame_coord, text="X COORDINATE (mm): ", font=("Arial", 8))
        self.lbl_x_coord.grid(column=0, row=0, sticky=W)

        self.ent_x_coord = Entry(frame_coord, width=10)
        self.ent_x_coord.grid(column=1, row=0)
        self.ent_x_coord.focus()

        self.lbl_y_coord = Label(frame_coord, text="Y COORDINATE (mm): ", font=("Arial", 8))
        self.lbl_y_coord.grid(column=0, row=1, sticky=W)

        self.ent_y_coord = Entry(frame_coord, width=10)
        self.ent_y_coord.grid(column=1, row=1)

        self.btn_compute = Button(frame_coord, text="Compute", command=lambda: self.clickedComputeA(scaramotor), font=("Arial", 8))
        self.btn_compute.grid(column=2, row=0, rowspan=2, sticky=W+N+S)

        self.lbl_coordmes = Label(frame_coord, font=("Arial", 8), fg='red')
        self.lbl_coordmes.grid(column=0, row=2, columnspan=3, sticky=W)

        frame_angle = Frame(master)
        frame_angle.grid(column=1, row=1, sticky=N)

        self.lbl_angle_1 = Label(frame_angle, text="ANGLE 1 (deg): ", font=("Arial", 8))
        self.lbl_angle_1.grid(column=0, row=0, sticky=W)

        self.ent_angle_1 = Entry(frame_angle, width=10)
        self.ent_angle_1.grid(column=1, row=0)

        self.lbl_angle_2 = Label(frame_angle, text="ANGLE 2 (deg): ", font=("Arial", 8))
        self.lbl_angle_2.grid(column=0, row=1, sticky=W)

        self.ent_angle_2 = Entry(frame_angle, width=10)
        self.ent_angle_2.grid(column=1, row=1)

        self.btn_send_angle = Button(frame_angle, text="  Send  ", command=lambda: self.clickedSendAngle(scaramotor, serial1), font=("Arial", 8))
        self.btn_send_angle.grid(column=2, row=0, rowspan=2, sticky=W+N+S)

    def clickedComputeA(self, scaramotor):
        if self.__is_float(self.ent_x_coord.get()) and self.__is_float(self.ent_y_coord.get()):
            pos = [float(self.ent_x_coord.get()), float(self.ent_y_coord.get())]
            self.lbl_coordmes.configure(text=str(pos))
            try:
                angle = scaramotor.find_angles(pos)
                self.ent_angle_1.delete(0, END)
                self.ent_angle_2.delete(0, END)
                self.ent_angle_1.insert(0, str(angle[0]))
                self.ent_angle_2.insert(0, str(angle[1]))
            except ValueError:
                self.lbl_coordmes.configure(text="Out of bounds")
        else:
            self.lbl_coordmes.configure(text="This is not a float")

    def clickedSendAngle(self, scaramotor, serial1):
        current_angles = scaramotor.getAngle()
        try:
            gotoangle = [float(self.ent_angle_1.get()), float(self.ent_angle_2.get())]
            steps = [scaramotor.steps_to_go(current_angles[0], gotoangle[0], scaramotor.getReso()[0]),
                     scaramotor.steps_to_go(current_angles[1], gotoangle[1], scaramotor.getReso()[1])]
            direc = [scaramotor.motor_dir(current_angles[0], gotoangle[0]),
                     scaramotor.motor_dir(current_angles[1], gotoangle[1])]
            scaramotor.updateAngle(
                [current_angles[0] + scaramotor.steps_to_angle(steps[0], direc[0], scaramotor.getReso()[0]),
                 current_angles[1] + scaramotor.steps_to_angle(steps[1], direc[1], scaramotor.getReso()[1])])
            serial1.send_start_byte()
            serial1.serial_send_steps_dir_woffset(steps, direc)
        except ValueError:
            print("ErrorAngle")


    def __is_float(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False


class LineInput:


    def __init__(self, master, scaramotor, serial1):
        self.line_solution = None

        frame_line = Frame(master)
        frame_line.grid(column=0, row=2, columnspan=2)

        self.lbl_x1 = Label(frame_line, text="X1 (mm): " + str(scaramotor.find_pos(scaramotor.getAngle())[0]), font=("Arial", 8))
        self.lbl_x1.grid(column=0, row=0, sticky=W)

        #self.ent_x1 = Entry(frame_line, width=5)
        #self.ent_x1.grid(column=1, row=0, sticky=W)

        self.lbl_y1 = Label(frame_line, text="Y1 (mm): " + str(scaramotor.find_pos(scaramotor.getAngle())[1]), font=("Arial", 8))
        self.lbl_y1.grid(column=0, row=1, sticky=W)

        #self.ent_y1 = Entry(frame_line, width=5)
        #self.ent_y1.grid(column=1, row=1, sticky=W)

        self.lbl_x2 = Label(frame_line, text="X2 (mm): ", font=("Arial", 8))
        self.lbl_x2.grid(column=2, row=0)

        self.ent_x2 = Entry(frame_line, width=5)
        self.ent_x2.grid(column=3, row=0)

        self.lbl_y2 = Label(frame_line, text="Y2 (mm): ", font=("Arial", 8))
        self.lbl_y2.grid(column=2, row=1)

        self.ent_y2 = Entry(frame_line, width=5)
        self.ent_y2.grid(column=3, row=1)

        self.lbl_speed = Label(frame_line, text="Speed (mm/s):", font=("Arial", 8))
        self.lbl_speed.grid(column=4, row=0)

        self.ent_speed = Entry(frame_line, width=10)
        self.ent_speed.grid(column=5, row=0)

        self.btn_comp_line = Button(frame_line, text="  Compute  ", command=lambda: self.clickedCompLine(scaramotor),
                                     font=("Arial", 8))
        self.btn_comp_line.grid(column=6, row=0, sticky=N+S+E)

        self.btn_send_linecmd = Button(frame_line, text="  Send  ", command=lambda: self.clickedSendLine(serial1, scaramotor),
                                       font=("Arial", 8))
        self.btn_send_linecmd.grid(column=6, row=1, sticky=W+ N + S + E)


    def clickedCompLine(self, scaramotor):
        pos1 = scaramotor.find_pos(scaramotor.getAngle())
        pos2 = [float(self.ent_x2.get()), float(self.ent_y2.get())]
        speed = float(self.ent_speed.get())
        self.line_solution = scaramotor.line_compute_times(pos1, pos2, speed)
        print(self.line_solution)
        print(self.line_solution[3][0][1])

    def clickedSendLine(self, serial1, scaramotor):
        current_angles = scaramotor.getAngle()
        steps = [int(self.line_solution[2][0]), int(self.line_solution[2][1])]
        direc = [self.line_solution[3][0][0], self.line_solution[3][1][0]]
        turn_values = [self.line_solution[3][0][1], self.line_solution[3][1][1]]

        serial1.send_start_byte_line()
        #16 bytes
        serial1.send_floats_serial(self.line_solution[0]*1000000)
        #16 bytes
        serial1.send_floats_serial(self.line_solution[1]*1000000)
        #5 bytes
        serial1.serial_send_steps_dir(steps, direc)
        #4 bytes
        serial1.serial_send_two_int(turn_values)

        scaramotor.updateAngle([current_angles[0] + scaramotor.steps_to_angle(steps[0]-2.0*turn_values[0], direc[0], scaramotor.getReso()[0]),
                 current_angles[1] + scaramotor.steps_to_angle(steps[1]-2.0*turn_values[1], direc[1], scaramotor.getReso()[1])])

        self.lbl_x1.configure(text = "X1 (mm): " + str(scaramotor.find_pos(scaramotor.getAngle())[0]))
        self.lbl_y1.configure(text = "Y1 (mm): " + str(scaramotor.find_pos(scaramotor.getAngle())[1]))

            #scaramotor.find_discrete_angle([float(self.ent_x2.get()), float(self.ent_y2.get())]))


if __name__ == "__main__":
    main()


