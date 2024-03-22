from tkinter import*
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

#Create GUI Window with title
mygui = Tk()
mygui.title('Articulated Calculator')
mygui.resizable(False,False)
mygui.configure(bg='#F4A261')


def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0,END)

    T1_E.delete(0,END)
    T2_E.delete(0,END)
    T3_E.delete(0,END)

    Z_E.delete(0,END)
    Y_E.delete(0,END)
    X_E.delete(0,END)

def f_k():
#link lengths in cm
    a1 = float(a1_E.get())/100
    a2 = float(a2_E.get())/100
    a3 = float(a3_E.get())/100

#joint variables in cm
    T1 = float(T1_E.get())
    T2 = float(T2_E.get())
    T3 = float(T3_E.get())

# degree to radian
    T1 = (T1/180.0)*np.pi
    T2 = (T2/180.0)*np.pi
    T3 = (T3/180.0)*np.pi

# Parametric Table (theta, alpha, r, d)
    PT = [[T1,(90.0/180.0)*np.pi,0,a1],
          [T2,(0.0/180.0)*np.pi,a2,0],
          [T3,(0.0/180.0)*np.pi,a3,0]]

# HTM formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]), np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

    H0_1 = np.matrix (H0_1)
    H1_2 = np.matrix (H1_2)
    H2_3 = np.matrix (H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)

    X0_3 = H0_3[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3*100,3))

#Create links
#[robot_value]=DHRobot([RevoluteDH(d,r,alpha,offset)])

    Articulated_Group6_Calculator=DHRobot([
        RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])],name='Articulated')

#plot joints
    q1 = np.array([T1,T2,T3])

#plot scale
    x1=-0.5
    x2=0.5
    y1=-0.5
    y2=0.5
    z1=0.0
    z2=0.5

#Plot Command

    Articulated_Group6_Calculator.plot(q1,limits=[x1,x2,y1,y2,z1,z2])

def i_k():
#link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
#position vector in cm
    x0_3 = float(X_E.get())
    y0_3 = float(Y_E.get())
    z0_3 = float(Z_E.get())

    if x0_3 == 0:
         theta1 = np.pi/2 if y0_3 > 0 else -np.pi/2
    else:
         theta1 = np.arctan(y0_3/x0_3) #Solution1

    r1 = np.sqrt(y0_3**2 + x0_3**2) #Solution2
    r2 = z0_3-a1 #Solution3

    if r1 == 0:
         phi1 = np.pi/2 if r2 > 0 else - np.pi/2
    else:
         phi1 = np.arctan(r2/r1) #Solution4

    r3 = np.sqrt(r2**2+r1**2) #Solution5

    phi2 = np.arccos(np.clip((a3**2-a2**2-r3**2)/(-2*a2*r3),-1,1)) #Solution6

    theta2 = phi1+phi2 #Solution7

    phi3 = np.arccos(np.clip((r3**2-a2**2-a3**2)/(-2*a2*a3),-1,1)) #Solution8

    theta3 = phi3-np.pi #Solution9

    T1_E.delete(0,END)
    T1_E.insert(0,np.around(theta1*180/np.pi,3))

    T2_E.delete(0,END)
    T2_E.insert(0,np.around(theta2*180/np.pi,3))

    T3_E.delete(0,END)
    T3_E.insert(0,np.around(theta3*180/np.pi,3))

#Create links
#[robot_value]=DHRobot([RevoluteDH(d,r,alpha,offset)])
        
    Articulated_Group6_Calculator=DHRobot([
        RevoluteDH(a1/100,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a2/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        RevoluteDH(0,a3/100,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])],name='Articulated')

#plot joints
    q1 = np.array([theta1,theta2,theta3])

#plot scale
x1=-0.5
x2=0.5
y1=-0.5
y2=0.5
z1=0.0
z2=0.5

#Link lengths and Joint Variable Frame
FI=LabelFrame(mygui,text='Link Lengths and Joint Variables',font=(5),bg='#F4A261')
FI.grid(row=0,column=0)

#Link Lenghts Label
a1 = Label(FI,text=('a1='),font=(5),bg='#2A9D8F')
a1_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
cm1 =  Label(FI,text=('cm'),font=(5),bg='#2A9D8F')

a2 = Label(FI,text=('a2='),font=(5),bg='#2A9D8F')
a2_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
cm2 =  Label(FI,text=('cm'),font=(5),bg='#2A9D8F')

a3 = Label(FI,text=('a3='),font=(5),bg='#2A9D8F')
a3_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
cm3 =  Label(FI,text=('cm'),font=(5),bg='#2A9D8F')

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

#Joint Variable Label
T1 = Label(FI,text=('T1 = '),font=(5),bg='#2A9D8F')
T1_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
deg1 = Label(FI,text=('deg'),font=(5),bg='#2A9D8F')

T2 = Label(FI,text=('T2 = '),font=(5),bg='#2A9D8F')
T2_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
deg2 = Label(FI,text=('deg'),font=(5),bg='#2A9D8F')

T3 = Label(FI,text=('T3 = '),font=(5),bg='#2A9D8F')
T3_E = Entry(FI,width=5,font=(5),bg='#E9C46A')
deg3 = Label(FI,text=('deg'),font=(5),bg='#2A9D8F')

T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

T3.grid(row=2,column=3)
T3_E.grid(row=2,column=4)
deg3.grid(row=2,column=5)

#Buttons Frame
BF = LabelFrame(mygui,text='Forward Kinematics',font=(5))
BF.grid(row=1,column=0)

#Buttons
FK = Button(BF,text='Forward',font=(5),bg='blue',fg='white',command=f_k)
rst = Button(BF,text='Reset',font=(5),bg='red',fg='white',command=reset)
IK = Button(BF,text='Inverse',font=(5),bg='green',fg='white',command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

#Position Vectors Frame
PV = LabelFrame(mygui,text='Position Vector',font=(5))
PV.grid(row=2,column=0)

#Position Vector Label
X = Label(PV,text=('X = '),font=(5),bg='#2A9D8F')
X_E = Entry(PV,width=5,font=(5))
cm4 =  Label(PV,text=('cm'),font=(5),bg='#2A9D8F')

Y = Label(PV,text=('Y = '),font=(5),bg='#2A9D8F')
Y_E = Entry(PV,width=5,font=(5))
cm5 =  Label(PV,text=('cm'),font=(5),bg='#2A9D8F')

Z = Label(PV,text=('Z = '),font=(5),bg='#2A9D8F')
Z_E = Entry(PV,width=5,font=(5))
cm6 =  Label(PV,text=('cm'),font=(5),bg='#2A9D8F')

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm4.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm5.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm6.grid(row=2,column=2)

#insert image
img = PhotoImage(file='Articulated.png')
img = img.subsample(3,3)
PI = Label(mygui,image=img)
PI.grid(row=3,column=0)

mygui.mainloop()