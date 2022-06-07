import sys
import numpy as np
import matplotlib.pyplot as plt
from pylab import *
from scipy.optimize import leastsq
import re

def get_data(logpath):
    Datas=[]
    with open(logpath,'r',encoding='ANSI',errors='ignore') as f:
        for line in f:
            line.strip('\n')
            if (re.search("判断采集结束",line)):
                break;
            if (re.search("收到",line)):
                line=line[line.find("Laster:")+7:-1]
                line=line.replace("\t"," ")
                line=re.findall(r"\d+",line)
                Datas.append(line)
    for i in range(len(Datas)):
        for j in range(len(Datas[0])):
            Datas[i][j]=int(Datas[i][j])
    while(not Datas[0][-3]^Datas[0][-4]):
        Datas.remove(Datas[0])
        if (len(Datas)==0):
            return Datas
    while(not Datas[0][-4]):
        Datas.remove(Datas[0])
        if (len(Datas)==0):
            return Datas
    if Datas[0][1]>Datas[-1][1]:
        Datas.reverse()
    return Datas
    
    # 二次函数的标准形式
def func(params, x):
    a, b, c = params
    return a * x * x + b * x + c
 
# 误差函数，即拟合曲线所求的值与实际值的差
def error(params, x, y):
    return func(params, x) - y
 
# 对参数求解
def slovePara(X,Y):
    p0 = [10, 10, 10]
    Para = leastsq(error, p0, args=(X, Y))
    return Para

# 输出最后的结果
def solution(X,Y):
    Para = slovePara(X,Y)
    a, b, c = Para[0]
    print ("a=",a," b=",b," c=",c,"-b/2a=",-b/(2*a))
    print ("cost:" + str(Para[1]))
    print ("求解的曲线是:")
    print("y="+str(round(a,2))+"x*x+"+str(round(b,2))+"x+"+str(c))

    plt.figure()
    plt.scatter(X, Y, color="green", label="sample data", linewidth=2)
    X_line=X[int(len(X)/2)]
    Y_line=Y[int(len(Y)/2)]
    plot([X[0],X_line],[Y_line,Y_line],'b')
    plt.ylabel(Y_line)
    plt.xlabel(X_line)

    #画拟合直线
    x=np.linspace(X[0],X[-1],100) ##在0-15直接画100个连续点
    y=a*x*x+b*x+c ##函数式
    plt.plot(x,y,color="red",label="solution line",linewidth=2)
    plt.legend() #绘制图例
    plt.show()
    Errors=np.zeros((1,len(Y)))
    for j in range(len(Errors[0])):
        Errors[0][j]=Y[j]-func([a,b,c],X[j])
    r2=1-np.var(Errors[0])/np.var(Y)
    return r2
    
logpaths=input("log file path and name:")
logpaths.replace("\\","/")
Data=get_data(logpaths)

if (len(Data)==0):
    print("No Datas!")
x=[]
x_modi=[]
wh=[]
ax=[]
for i in range(len(Data)):
    x.append(Data[i][1])
    x_modi.append(Data[i][1]+i*50)
    ax.append(Data[i][0])
    wh.append(Data[i][-4])
    
#!!!!!añadir un proceso de reserva!!!!!

descenso=np.where(np.array(ax)<ax[0])
minimos=np.where(np.diff(descenso)!=1)
r2_compare=0
eje_location=0

if (len(descenso[0])==0):
    print("Data error!")
    
if len(minimos[1])==0:
    minimo=descenso[0][int(len(descenso[0])/2)]
    if minimo<20 or len(x)-minimo<20:
        mlong=minimo
    else:
        mlong=20
    Xs=np.array(x_modi[minimo-mlong:minimo+mlong])
    Ys=np.array(ax[minimo-mlong:minimo+mlong])
    r2=solution(Xs,Ys)
    if r2>r2_compare:
        r2_compare=r2
        eje_location=minimo
else:
    for i in range(len(minimos[1])+1):
        if i==0:
            minimo=descenso[0][int((minimos[1][i]+1)/2)]
        else:
            if i==len(minimos[1]):
                minimo=descenso[0][int(((minimos[1][-1])+len(descenso[0]))/2)]
            else:
                minimo=descenso[0][int((minimos[1][i]+minimos[1][i-1])/2)]
        if wh[minimo]==0:
            continue
        if minimo<30:
            mlong=minimo
            Xs=np.array(x_modi[minimo-mlong:minimo+mlong*2])
            Ys=np.array(ax[minimo-mlong:minimo+mlong*2])
        else:
            mlong=30
            Xs=np.array(x_modi[minimo-mlong:minimo+mlong])
            Ys=np.array(ax[minimo-mlong:minimo+mlong])

        r2=solution(Xs,Ys)
        if r2>r2_compare:
            r2_compare=r2
            eje_location=minimo
print(r2_compare,eje_location)

fig,axie1=plt.subplots()
axie1.plot(x,ax,'r')
axie1.plot(x[eje_location],ax[eje_location],'ko')
axie2=axie1.twinx()
axie2.plot(x,wh)
axie1.set_xlabel(x[eje_location])
axie1.set_ylabel(ax[eje_location])

plt.show()
