from tkinter import *
import tkinter.ttk as ttk
import serial as ser
import serial.tools.list_ports

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


def clickedComputeA():
    pass


def clickedSendAngle():
    pass




window = Tk()
window.title("SCARA GUI")
#window.geometry("350x250")

serial1 = SerialConnection(window)

frame_coord = Frame(window)
frame_coord.grid(column=0, row=1)

lbl_x_coord = Label(frame_coord, text="X COORDINATE: ", font=("Arial", 8))
lbl_x_coord.grid(column=0, row=0, sticky=W)

ent_x_coord = Entry(frame_coord, width=10)
ent_x_coord.grid(column=1, row=0)
ent_x_coord.focus()

lbl_y_coord = Label(frame_coord, text="Y COORDINATE: ", font=("Arial", 8))
lbl_y_coord.grid(column=0, row=1, sticky=W)

ent_y_coord = Entry(frame_coord, width=10)
ent_y_coord.grid(column=1, row=1)

btn_compute = Button(frame_coord, text="Compute", command=clickedComputeA, font=("Arial", 8))
btn_compute.grid(column=2, row=0, rowspan=2, sticky=W+N+S)


frame_angle = Frame(window)
frame_angle.grid(column=1, row=1)

lbl_angle_1 = Label(frame_angle, text="ANGLE 1: ", font=("Arial", 8))
lbl_angle_1.grid(column=0, row=0, sticky=W)

ent_angle_1 = Entry(frame_angle, width=10)
ent_angle_1.grid(column=1, row=0)

lbl_angle_2 = Label(frame_angle, text="ANGLE 2: ", font=("Arial", 8))
lbl_angle_2.grid(column=0, row=1, sticky=W)

ent_angle_2 = Entry(frame_angle, width=10)
ent_angle_2.grid(column=1, row=1)

btn_send_angle = Button(frame_angle, text="Compute", command=clickedSendAngle, font=("Arial", 8))
btn_send_angle.grid(column=2, row=0, rowspan=2, sticky=W+N+S)

window.mainloop()