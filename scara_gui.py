from tkinter import *
import tkinter.ttk as ttk
import serial as ser


window = Tk()
window.title("SCARA GUI")
window.geometry("500x350")


def clicked():
    res = "My name is " + txt.get() + combo.get()
    label.configure(text=res)


label = Label(window, text="Hello World!!", font=("Arial", 12))
label.grid(column=0, row=0)

btn = Button(window, text="Click Me", command=clicked, font=("Comic Sans MS", 11), bg='blue', fg='red')
btn.grid(column=1, row=0)

txt = Entry(window, width=10)
txt.grid(column=0, row=1)
txt.focus()

combo = ttk.Combobox(window)
combo['values'] = (1, 2, 3, 4, 5, "Text")
combo.current(1)
combo.grid(column=0, row=2)


window.mainloop()