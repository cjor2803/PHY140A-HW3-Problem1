#Packages
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import PillowWriter
import math

#Inputs
def vectorin():
    vector=np.array([])
    for i in range(3):
        vector=np.append(vector,float(input()))
    return vector
print("Welcome to Charles' Fun-Time with Miller Indices!")
print("Note to User: This program is in it's beta...")
print("              While it will plot any plane, the lattice points will not be right unless the lattice is primitive cubic.")
print("              And there may be many, many more bugs")
print('Please enter the first primitive translation vector, a1.')
print('Press "enter" after submitting each component')
a1=vectorin()
print("Now a2")
a2=vectorin()
print("And a3")
a3=vectorin()
print("Finally, enter each of the Miller Indices:")
M=vectorin()
Scale=float(input('How much should the distance between the planes be scaled? '))
Density=int(input('Now input the lattice scarsity as a positive integer (Higher number->Less lattice points): '))

#Reciprocal Lattice
a2xa3=np.cross(a2,a3)
V=np.dot(a1,a2xa3)
b1=2*np.pi*a2xa3/V
b2=2*np.pi*np.cross(a3,a1)/V
b3=2*np.pi*np.cross(a1,a2)/V
Gvec=M[0]*b1+M[1]*b2+M[2]*b3

#Plane
def Plane(a,b,c,Shift):
    Offsetvec=2*np.pi*Gvec*Shift
    if a!=0:
        Y=np.linspace(0,10,100)
        Z=np.linspace(0,10,100)
        Y,Z= np.meshgrid(Y, Z)
        X=-(b*Y+c*Z+Offsetvec[0])/a
    elif b!=0:
        X=np.linspace(0,10,100)
        Z=np.linspace(0,10,100)
        X,Z= np.meshgrid(X, Z)
        Y=-(a*X+c*Z+Offsetvec[1])/b
    else:
        X=np.linspace(0,10,100)
        Y=np.linspace(0,10,100)
        X,Y= np.meshgrid(X, Y)
        Z=-(a*X+b*Y+Offsetvec[2])/c
    return X,Y,Z

#Plot Plane
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

X,Y,Z=Plane(Gvec[0],Gvec[1],Gvec[2],0)
surf = ax.plot_surface(X, Y, Z)
Px,Py,Pz= Plane(Gvec[0],Gvec[1],Gvec[2],1)
surf = ax.plot_surface(Px, Py, Pz)

MaxX=math.ceil(np.max(np.append(X,Px)))
MaxY=math.ceil(np.max(np.append(Y,Py)))
MaxZ=math.ceil(np.max(np.append(Z,Pz)))
MinX=math.floor(np.min(np.append(X,Px)))
MinY=math.floor(np.min(np.append(Y,Py)))
MinZ=math.floor(np.min(np.append(Z,Pz)))

boundary=np.array([MinX,MinY,MinZ,MaxX,MaxY,MaxZ])
#Lattice Points
Atomsx=np.array([])
Atomsy=np.array([])
Atomsz=np.array([])

def Boundary(A):
    for i in range(3):
        if A[i]<boundary[i]:
            return True
        elif A[i]>boundary[i+3]:
            return True
N=200

for i in range(-N,N,Density):
    A=i*a1
    if Boundary(A)==True:
        continue
    for j in range(-N,N,Density):
        B=A+j*a2
        if Boundary(B)==True:
            continue
        for k in range(-N,N,Density):
            C=B+k*a3
            if Boundary(C)==True:
                continue
            Atomsx=np.append(Atomsx,C[0])
            Atomsy=np.append(Atomsy,C[1])
            Atomsz=np.append(Atomsz,C[2])

            
ax.scatter(Atomsx, Atomsy, Atomsz)

#Plot
metadata=dict(title='Movie', artist='Charles Jordan')  
writer=PillowWriter(fps=15,metadata=metadata)  

ax.scatter(Atomsx, Atomsy, Atomsz)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
elev = azim = roll = 0
with writer.saving(fig, 'AwsomeMillerPlane.gif',100):
    for angle in range(0, 360):
        # Normalize the angle to the range [-180, 180] for display
        azim = (angle + 180) % 360 - 180

        # Update the axis view and title
        ax.view_init(elev, azim, roll)
        plt.title('Miller Plane (%d%d%d) Angle: %d°' % (M[0],M[1],M[2],azim))
        plt.draw()
        loading=angle/360
        Percent="{:.0%}".format(loading)
        print(Percent)
        writer.grab_frame()
print("Done! The gif has been stored as 'AwsomeMillerPlane.gif'.")