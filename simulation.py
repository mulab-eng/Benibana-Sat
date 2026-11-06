import numpy as np
import matplotlib.pyplot as plt

mu = 4.0   
a = 2.0    
b = 1.0

t0 = 0.0    #初期時刻[sec] 
tf = 100.0    #終わりの時刻[sec]
h = 0.01    #時間の刻み幅[sec]
N = round((tf-t0)/h)    #forの繰り返しの回数
T = np.zeros(N+1)     #時刻のデータを入れる配列
X = np.zeros(N+1)      #Xのデータを入れる配列
Y = np.zeros(N+1)      #Yのデータを入れる配列

T[0] = t0      # 時刻の第一要素に初期時刻 0.0 を代入
X[0] = 3.0     # xの初期値
Y[0] = 2.0     #ｙの初期値
for n in range(1,N+1):  #n=1から10000までの繰り返し
    T[n] = t0+n*h   #nを変えながら時刻の値を計算して代入
    dxdt = mu*X[n-1]-a*Y[n-1]-(X[n-1]**2+Y[n-1]**2)*(X[n-1]-b*Y[n-1])     #xの変化率dxdtを計算
    X[n] = X[n-1]+dxdt*h     #xの変化率dxdtと刻み時間hを用いてｈ秒後のxの値を計算
    dydt = a*X[n-1]+mu*Y[n-1]-(X[n-1]**2+Y[n-1]**2)*(b*X[n-1]+Y[n-1])     #yの変化率dydtを計算
    Y[n] = Y[n-1]+dydt*h     #yの変化率dydtと刻み時間hを用いてｈ秒後のyの値を計算
    
fig = plt.figure(figsize=(10,5))
plt.xlim(0,20)
plt.ylim(-2.5,2.5)
plt.plot(T, X,'-', label="$(T,X)$")
plt.plot(T, Y,'-', label="$(T,Y)$")
plt.title('Stuart Landau Equation')   #タイトル
plt.xlabel('time')    #x座標のラベル
plt.ylabel('x, y')    #y座標のラベル
plt.legend()
plt.show()
