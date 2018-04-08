from tkinter import *
import tkinter.ttk as ttk
import serial as ser
import serial.tools.list_ports
import line_compute_time
import math


class SerialConnection:

    def __init__(self, master):
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
        if self.btn_connect['text']=="Connect":
            global arduinosrl
            arduinosrl = ser.Serial(self.combo.get(), 9600)
            self.lbl_PortConnect.configure(text="Connected to " + self.combo.get())
            self.btn_connect.configure(text="Disconnect")
        else:
            arduinosrl.close()
            self.btn_connect.configure(text="Connect")
            self.lbl_PortConnect.configure(text="Disconnected")


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

        self.btn_compute = Button(frame_coord, text="Compute", command=self.clickedComputeA, font=("Arial", 8))
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

    def clickedComputeA(self):
        if self.__is_float(self.ent_x_coord.get()) and self.__is_float(self.ent_y_coord.get()):
            pos = [float(self.ent_x_coord.get()), float(self.ent_y_coord.get())]
            self.lbl_coordmes.configure(text=str(pos))
            try:
                angle = line_compute_time.find_angles(pos)
                self.ent_angle_1.insert(0, str(angle[0]))
                self.ent_angle_2.insert(0, str(angle[1]))
            except ValueError:
                self.lbl_coordmes.configure(text="Out of bounds")
        else:
            self.lbl_coordmes.configure(text="This is not a float")
        print("wha")

    def clickedSendAngle(self, scaramotor):
        current_angles = scaramotor.getAngle()
        try:
            gotoangle = [float(self.ent_angle_1.get()), float(self.ent_angle_2.get())]
            steps = [self.__steps_to_go(current_angles[0],gotoangle[0],scaramotor.getReso()[0]), self.__steps_to_go(current_angles[1],gotoangle[1],scaramotor.getReso()[1])]
            direc = self.__find_dir(current_angles, gotoangle)
            scaramotor.updateAngle(
                [current_angles[0] + scaramotor.steps_to_angle(steps[0], direc[0], scaramotor.getReso()[0]),
                 current_angles[1] + scaramotor.steps_to_angle(steps[1], direc[1], scaramotor.getReso()[1])])
        except ValueError:
            print("ErrorAngle")
        #print(scaramotor.getAngle())

    def __is_float(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False

    def __steps_to_go(self, angle_in, angle_fin, reso):
        return int(abs(angle_fin-angle_in)//reso)

    def __find_dir(self, cur_angles, gotoangle):
        direction = [0, 0]
        if gotoangle[0] >= cur_angles[0]:
            direction[0] = 1
        else:
            direction[0] = 0
        if gotoangle[1] >= cur_angles[1]:
            direction[1] = 1
        else:
            direction[1] = 0
        return direction




class ScaraMotors:

    def __init__(self, angle_init):
        self.angle_cur = angle_init
        self.resolution = [2*math.pi/200/3.70588235/8, 2*math.pi/400/2/8]

    def updateAngle(self, angle_updated):
        self.angle_cur = angle_updated

    def getAngle(self):
        return self.angle_cur

    def getReso(self):
        return self.resolution

    def steps_to_angle(self, steps, dir, reso):
        if dir:
            return steps*reso
        else:
            return -steps*reso


window = Tk()
window.title("SCARA GUI")
#window.geometry("350x250")
angle_in = [0,0]
scaramotor1 = ScaraMotors(angle_in)
serial1 = SerialConnection(window)
pointtopoint = PointtoPoint(window,scaramotor1, serial1)
window.mainloop()

