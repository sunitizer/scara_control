from tkinter import *
import tkinter.ttk as ttk
import serial
import serial.tools.list_ports
import math
import struct
import ScaraMotors

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

    def serial_send_steps_dir(self, steps, direc):
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
        self.send_start_byte()
        self.arduinosrl.write(dir_byte)
        self.arduinosrl.write(off_byte)
        self.arduinosrl.write(motor1_msb)
        self.arduinosrl.write(motor1_lsb)
        self.arduinosrl.write(motor2_msb)
        self.arduinosrl.write(motor2_lsb)

    def send_start_byte(self):
        self.arduinosrl.write(struct.pack('>B', 0b11111111))

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

        frame_angle = Frame(window)
        frame_angle.grid(column=1, row=1, sticky=N)

        self.lbl_angle_1 = Label(frame_angle, text="ANGLE 1 (deg): ", font=("Arial", 8))
        self.lbl_angle_1.grid(column=0, row=0, sticky=W)

        self.ent_angle_1 = Entry(frame_angle, width=10)
        self.ent_angle_1.grid(column=1, row=0)

        self.lbl_angle_2 = Label(frame_angle, text="ANGLE 2 (deg): ", font=("Arial", 8))
        self.lbl_angle_2.grid(column=0, row=1, sticky=W)

        self.ent_angle_2 = Entry(frame_angle, width=10)
        self.ent_angle_2.grid(column=1, row=1)

        self.btn_send_angle = Button(frame_angle, text="  Send  ", command=lambda: self.clickedSendAngle(scaramotor), font=("Arial", 8))
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

    def clickedSendAngle(self, scaramotor):
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
            serial1.serial_send_steps_dir(steps, direc)
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
        frame_line = Frame(master)
        frame_line.grid(column=0, row=2, columnspan=2)

        self.lbl_x1 = Label(frame_line, text="X1 (mm): ", font=("Arial", 8))
        self.lbl_x1.grid(column=0, row=0, sticky=W)

        self.ent_x1 = Entry(frame_line, width=5)
        self.ent_x1.grid(column=1, row=0, sticky=W)

        self.lbl_y1 = Label(frame_line, text="Y1 (mm): ", font=("Arial", 8))
        self.lbl_y1.grid(column=0, row=1, sticky=W)

        self.ent_y1 = Entry(frame_line, width=5)
        self.ent_y1.grid(column=1, row=1, sticky=W)

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

        self.btn_send_linecmd = Button(frame_line, text="  Send  ", command=lambda: self.clickedSendLine(scaramotor),
                                     font=("Arial", 8))
        self.btn_send_linecmd.grid(column=6, row=0, rowspan=2, sticky=N+S+E)

    def clickedSendLine(self, scaramotor):
        pass




window = Tk()
window.title("SCARA GUI")
#window.geometry("350x250")
angle_in = [0,0]
scaramotor1 = ScaraMotors.ScaraMotors(arm_len1=179.9, arm_len2=150, angle_res1=2*math.pi/200/3.70588235/8,
                          angle_res2=2*math.pi/400/2/8, angle_init=angle_in)
serial1 = SerialConnection(window)
pointtopoint = PointtoPoint(window, scaramotor1, serial1)
straight_line = LineInput(window, scaramotor1, serial1)
window.mainloop()

