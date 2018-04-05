from tkinter import *
import tkinter.ttk as ttk
import serial as ser
import serial.tools.list_ports


ports = serial.tools.list_ports.comports()
portlist = ['']
for p in ports:
    print(p.device)
    portlist.append(str(p.device))
#arduinoSrl = ser.Serial('COM3', 9600)


window = Tk()
window.title("SCARA GUI")
#window.geometry("350x250")

serFrame = Frame(window)
serFrame.pack(side=TOP)



def clickedConnectSrl():
    arduinoSrl = ser.Serial(combo.get(), 9600)
    lbl_PortConnect.configure(text="Connected to " + combo.get())


lbl_Srl = Label(serFrame, text="Serial Port:", font=("Arial", 8))
lbl_Srl.grid(column=0, row=0)

combo = ttk.Combobox(serFrame, values=portlist)
combo.current(0)
combo.grid(column=1, row=0)

btn = Button(serFrame, text="Connect", command=clickedConnectSrl, font=("Arial", 8))
btn.grid(column=2, row=0)

lbl_PortConnect = Label(serFrame, font=("Arial", 8))
lbl_PortConnect.grid(columnspan=3)
#lbl_PortConnect.grid(column=0, row=1)


txt = Entry(window, width=10)
txt.pack()
#txt.grid(column=0, row=2)
txt.focus()




window.mainloop()