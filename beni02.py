import pygame
import sys
import math
import random
import os

############################### ウィンドウの大きさ　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　
WIDTH=800
HEIGHT=640

########################### 画像ファイルを読み込む
img_messe = pygame.image.load("./image/pink.png")
img_messe2 = pygame.image.load("./image/reply.png")
img_satellite = pygame.image.load("./image/sat.png")
img_beni = pygame.image.load("./image/beni.png")
img_receive= pygame.image.load("./image/receive.png")
img_receive2= pygame.image.load("./image/receive2.png")
img_wall = pygame.image.load("./image/wall.png")
img_haikei=pygame.image.load("./image/earth.png")
img_haikei0=pygame.image.load("./image/haikei0.png")
img_start=pygame.image.load("./image/start2.jpg")
img_gameover=pygame.image.load("./image/gameover.png")
img_high=pygame.image.load("./image/high.png")

se_shot=None # 音声ファイル用変数
se_hit=None
se_hit3=None

############################### スコア
score=100

# count=0
ta=0

written=0
switch=0

scorefile=open('score01.txt','r')
zenkaiscore_str = scorefile.read()
if len(zenkaiscore_str) < 1:
    zenkaiscore_str='-100'
zenkaiscore = int(zenkaiscore_str)
scorefile.close()

scorefile=open('score01.txt','w')

############################### 地上局のメッセージの変数
e_msl_f=False
e_msl_x=300       # メッセージのx座標
e_msl_y=300       # メッセージのy座標
e_msl_theta=0   # メッセージの角度

################################ 地上局の変数
emy_x=200
emy_y=200
emy_vx=-40
emy_vy=-30
emy_theta=(5/4)*math.pi
emy_v=0
emy_omega=0.0

############################## 人工衛星の変数
Radius=300  # 軌道の半径
R_theta=0  # 軌道の角度
R_Omega=0.1  # 軌道の角速度
xr=-150  # 人工衛星のx座標
yr=-150   # 人工衛星の y座標
vx= 40  # x方向の速度（初期値）
vy= 30  # y方向の速度-
theta=1 # 人工衛星の角度の初期値[rad]
v=10   #[m/sec]  # 前進方向の速度
omega=-0.5 #[rad/sec]  # 回転の角速度

##################  雲の変数
w_xr=2*WIDTH #雲のx座標
w_yr=2*HEIGHT #雲のy座標
w_v=400 #雲の速さ
w_theta=0 #雲の角度
w_f=0 #雲のフラグ
w_size=0.25 #雲の縮尺
w_hp=2000 #雲の耐久値

################################### 画面の更新
deltaT=1/30  # 画面更新の刻み時間 Δt
timer=0  # 時間を測る変数

# ゲームの開始，プレイ中，ゲームオーバーを管理する変数
idx=0  
########## ジョイスティック（ゲームパッド）の有無
Joys=0  #初期値は無し

######## 複数のメッセージを送信できるようにする ########
message_MAX = 100
msl_no = 0
msl_f = [False]*message_MAX
msl_x = [0]*message_MAX
msl_y = [0]*message_MAX
msl_theta=[0]*message_MAX  # メッセージの角度
key_spc = 0
joy_b = 0

##############################################   画像化した文字を表示   ########################################################## 
def disp_str(screen, img, x, y, th, k):  # 画像を位置(x,y)，角度thで表示する関数．
                                        #  imgはファイル名，kは拡大縮小の倍率
    X= WIDTH*0.5 + x   # 座標変換
    Y= HEIGHT*0.5 - y # 座標変換
    th=th*180/math.pi # 座標変換
    TH=th # 座標変換
    img1 = pygame.transform.rotozoom(img, TH, k)  # 画像の回転と拡大縮小
    X = X - img1.get_width()/2    # 位置の微調整
    Y = Y - img1.get_height()/2   # 位置の微調整
    screen.blit(img1, [X, Y])     # 画像の表示
##############################################   画像表示   ########################################################## 
def disp_img(screen, img, x, y, th, k):  # 画像を位置(x,y)，角度thで表示する関数．
                                        #  imgはファイル名，kは拡大縮小の倍率
    X= WIDTH*0.5 + x   # 座標変換
    Y= HEIGHT*0.5 - y # 座標変換
    th=th*180/math.pi # 座標変換
    TH=th-90 # 座標変換
    img1 = pygame.transform.rotozoom(img, TH, k)  # 画像の回転と拡大縮小
    X = X - img1.get_width()/2    # 位置の微調整
    Y = Y - img1.get_height()/2   # 位置の微調整
    screen.blit(img1, [X, Y])     # 画像の表示

##############################################   メッセージ   ########################################################## 
def set_message(): # メッセージの初期設定
    global xr, yr, vx, vy, theta, v, omega, deltaT, msl_no
    global msl_f, msl_x, msl_y, msl_theta
    if msl_f[msl_no] == False:  # メッセージが存在しなければ
        msl_f[msl_no]= True     # メッセージを存在させて
        msl_x[msl_no] = xr+20*math.cos(theta) # メッセージの送信位置のx座標を計算（人工衛星の近く）
        msl_y[msl_no] = yr+20*math.sin(theta) # メッセージの送信位置のy座標を計算（人工衛星の近く）
        msl_theta[msl_no]=theta  # 送信するときのメッセージの角度を人工衛星の角度と同じに設定
        msl_no = (msl_no+1)%message_MAX

def e_set_message(): # 地上局のメッセージの初期設定
    global emy_x, emy_y, emy_vx, emy_vy, emy_theta, emy_v, emy_omega, deltaT 
    global e_msl_f, e_msl_x, e_msl_y, e_msl_theta
    if e_msl_f == False:  # 地上局のメッセージが存在しなければ
        e_msl_f= True     # 地上局のメッセージを存在させて
        e_msl_x = emy_x+15*math.cos(emy_theta) # 地上局のメッセージの送信位置のx座標を計算（人工衛星の近く）
        e_msl_y = emy_y+15*math.sin(emy_theta) # 地上局のメッセージの送信位置のy座標を計算（人工衛星の近く）
        e_msl_theta=emy_theta  # 送信するときの地上局のメッセージの角度を地上局の角度と同じに設定

def move_message(screen):  # メッセージを動かす関数
    global xr, yr, vx, vy, theta, v, omega, deltaT 
    global msl_f, msl_x, msl_y, msl_theta
    for i in range(message_MAX):
        if msl_f[i] == True:
            msl_x[i] = msl_x[i]  + 15*math.cos(msl_theta[i]) # メッセージのx座標を更新
            msl_y[i] = msl_y[i]  + 15*math.sin(msl_theta[i]) # メッセージのy座標を更新
            if math.fabs(msl_x[i]) > WIDTH*0.5: # メッセージのx座標の絶対値がウィンドウの半分を超えたら
                msl_f[i]=False # メッセージはウィンドウから出るので存在しない（False）とする
            if math.fabs(msl_y[i]) > HEIGHT*0.5: # メッセージのy座標の絶対値がウィンドウの半分を超えたら
                msl_f[i]=False # メッセージはウィンドウから出るので存在しない（False）とする
            disp_img(screen, img_messe, msl_x[i], msl_y[i], msl_theta[i], 0.15) # メッセージの画像を位置と角度を指定して表示

def e_move_message(screen):  # 地上局のメッセージを動かす関数
    global emy_x, emy_y, emy_vx, emy_vy, emy_theta, emy_v, emy_omega, deltaT 
    global e_msl_f, e_msl_x, e_msl_y, e_msl_theta
    if e_msl_f == True:
        e_msl_x = e_msl_x  + 10*math.cos(e_msl_theta) # 返信メッセージのx座標を更新
        e_msl_y = e_msl_y  + 10*math.sin(e_msl_theta) # メッセージのy座標を更新
        if math.fabs(e_msl_x) > WIDTH*0.5: # メッセージのx座標の絶対値がウィンドウの半分を超えたら
            e_msl_f=False # メッセージはウィンドウから出るので存在しない（False）とする
        if math.fabs(e_msl_y) > HEIGHT*0.5: # メッセージのy座標の絶対値がウィンドウの半分を超えたら
            e_msl_f=False # メッセージはウィンドウから出るので存在しない（False）とする
        disp_img(screen, img_messe2, e_msl_x, e_msl_y, e_msl_theta, 0.15) # メッセージの画像を位置と角度を指定して表示


##############################################   人工衛星   ########################################################## 
def move_satellite(screen):  # 人工衛星を動かす関数
    global xr, yr, vx, vy, theta, v, omega, deltaT, score, R_Omega, R_theta
    vx=v*math.cos(theta)   # 人工衛星のx方向の速度
    vy=v*math.sin(theta)   # 人工衛星のy方向の速度
    # vx=-Radius*R_Omega*math.sin(R_theta)   # 人工衛星のx方向の速度
    # vy=Radius*R_Omega*math.cos(R_theta)   # 人工衛星のy方向の速度
    # R_theta=R_theta+R_Omega*deltaT  # 軌道の角度を更新
    thetadot=omega         # 人工衛星の角速度
    xr = xr + vx * deltaT   # 人工衛星のx座標を更新
    yoyu=30
    if xr > WIDTH*0.5-yoyu:
        xr= WIDTH*0.5-yoyu
    if xr < -WIDTH*0.5+yoyu:
        xr= -WIDTH*0.5+yoyu
    yr = yr + vy * deltaT   # 人工衛星のy座標を更新
    if yr > HEIGHT*0.5-yoyu:
        yr= HEIGHT*0.5-yoyu
    if yr < -HEIGHT*0.5+yoyu:
        yr= -HEIGHT*0.5+yoyu
    theta = theta + thetadot * deltaT  # 人工衛星の角度を更新
    disp_img(screen, img_satellite, xr, yr, theta, 0.15) # 人工衛星の画像を位置と角度を指定して表示
    kyori2=(xr-e_msl_x)*(xr-e_msl_x)+(yr-e_msl_y)*(yr-e_msl_y)
    kyori=math.sqrt(kyori2)
    if kyori < 40:
        #衛星に当たった時
        se_hit.play()
        disp_img(screen, img_receive, xr, yr, theta, 0.25)
        score = score + 5
        # score = score-1
        # count = count +1
        
##############################################  地上局 ########################################################## 
def move_beni(screen):  # 地上局を動かす関数
    global emy_x, emy_y, emy_vx, emy_vy, emy_theta, emy_v, emy_omega, deltaT, score, count
    emy_vx=emy_v*math.cos(emy_theta)   # 地上局のx方向の速度
    emy_vy=emy_v*math.sin(emy_theta)   # 地上局のy方向の速度
    # emy_thetadot=emy_omega         # 地上局の角速度
    emy_x = emy_x + emy_vx * deltaT   # 地上局のx座標を更新
    emy_y = emy_y + emy_vy * deltaT   # 地上局のy座標を更新
    # emy_theta = emy_theta + emy_thetadot * deltaT  # 地上局の角度を更新
    
    e_kyori2=(xr-emy_x)*(xr-emy_x)+(yr-emy_y)*(yr-emy_y)
    e_kyori=math.sqrt(e_kyori2)
    if e_kyori > 270:
        move = ["front","stop"]
        choice = random.choice(move)
        if timer%15==0:
            if choice=="front":
                emy_v=30
            if choice=="stop":
                emy_v=0
    if e_kyori < 270:
        emy_v=-40

    ahead_t= 2.2 # 予測時間(秒) 好みに応じて調整
    next_xr = xr + vx * ahead_t
    next_yr = yr + vy * ahead_t
    dx = next_xr - emy_x
    dy = next_yr - emy_y

    if emy_theta > 2*math.pi:
        emy_theta = emy_theta - 2*math.pi
    if emy_theta < 0:
        emy_theta = emy_theta + 2*math.pi
    r_thetarad=math.atan2(dy,dx)  # 地上局から人工衛星へ引いたベクトルのラジアンの角度
    if r_thetarad < 0:
        r_thetarad = r_thetarad + 2*math.pi

    e=(emy_theta-r_thetarad+math.pi)%(2*math.pi)-math.pi
    gain=-1.0
    emy_thetadot=gain*e
    emy_theta = emy_theta + emy_thetadot * deltaT

    disp_img(screen, img_beni, emy_x, emy_y, emy_theta-1.6, 0.24) # 地上局の画像を位置と角度を指定して表示

    if emy_x > WIDTH*0.5:
        emy_x = WIDTH*0.5
    if emy_x < -WIDTH*0.5:
        emy_x = -WIDTH*0.5
    if emy_y > HEIGHT*0.5:
        emy_y = HEIGHT*0.5
    if emy_y < -HEIGHT*0.5:
        emy_y = -HEIGHT*0.5    

    for i in range(message_MAX):
        if msl_f[i] == True:
            kyori2=(emy_x-msl_x[i])*(emy_x-msl_x[i])+(emy_y-msl_y[i])*(emy_y-msl_y[i])
            kyori=math.sqrt(kyori2)
            if kyori < 40:
                # count = count +1
                #地上局に当たった時
                se_hit3.play()
                disp_img(screen, img_receive2, emy_x, emy_y, emy_theta, 0.2)
                msl_f[i]=False
                kyori3 = (emy_x-xr)*(emy_x-xr)+(emy_y-yr)*(emy_y-yr) 
                kyori4 = math.sqrt(kyori3) 
                score=score + 2
                e_set_message() #メッセージの送信準備
            # e_move_message(screen) #送信されたメッセージの移動と画像表示

##############################################   雲   ########################################################## 
def move_wall(screen,timer): #雲を動かす関数
    global w_xr,w_yr,w_v,w_theta,w_hp,deltaT,w_f,w_size
    global emy_x, emy_y, xr, yr, score
    global e_msl_f,e_msl_x,e_msl_y
    if w_hp>0: #雲の耐久値がある状態で実行
        x_kyori=math.fabs(emy_x-xr) #ロボットと敵のx方向距離
        y_kyori=math.fabs(emy_y-yr) #ロボットと敵のy方向距離
        if x_kyori<200 and y_kyori<200: #両方向の距離が100未満で実行
            if x_kyori<y_kyori:
                if w_f==0:
                    w_v=400
                    w_xr=xr
                    w_theta=0
                    if yr>0:
                        w_yr=HEIGHT*0.5
                        w_f=1 
                    else:
                        w_yr=-HEIGHT*0.5
                        w_f=2 
            if x_kyori>y_kyori:
                if w_f==0:
                    w_v=400
                    w_yr=yr
                    w_theta=math.pi*0.5
                    if xr>0:
                        w_xr=WIDTH*0.5
                        w_f=3 
                    else:
                        w_xr=-WIDTH*0.5
                        w_f=4 
        sec = timer*deltaT #timerの時間を秒に直す
        interval=20 #雲が降ってくる間隔
        if sec%interval==0 and w_f==0:
            w_v=100
            if math.fabs(xr)<math.fabs(yr):
                w_xr=xr
                w_theta=0
                if yr>0:
                    w_yr=HEIGHT*0.5
                    w_f=1 
                else:
                    w_yr=-HEIGHT*0.5
                    w_f=2 
            if math.fabs(xr)>=math.fabs(yr):
                w_yr=yr
                w_theta=math.pi*0.5
                if xr>0:
                    w_xr=WIDTH*0.5
                    w_f=3 
                else:
                    w_xr=-WIDTH*0.5
                    w_f=4 
        if w_f==1: #画面の上端から雲が出てくる
                disp_img(screen, img_wall, w_xr, w_yr, w_theta, w_size)
                w_yr-=w_v*deltaT
        if w_f==2: #画面の下端から雲が出てくる 
                disp_img(screen, img_wall, w_xr, w_yr, w_theta, w_size)
                w_yr+=w_v*deltaT
        if w_f==3: #画面の右端から雲が出てくる 
                disp_img(screen, img_wall, w_xr, w_yr, w_theta, w_size)
                w_xr-=w_v*deltaT
        if w_f==4: #画面の左端から雲が出てくる
                disp_img(screen, img_wall, w_xr, w_yr, w_theta, w_size)
                w_xr+=w_v*deltaT
        if math.fabs(w_xr)>WIDTH*0.5 or math.fabs(w_yr)>HEIGHT*0.5:
            w_f=0 #雲が画面外に出た場合
            w_xr=2*WIDTH #x座標初期値
            w_yr=2*HEIGHT #y座標初期値
        for i in range(message_MAX):
            if msl_f[i] == True:
                kyori_r=(w_xr-msl_x[i])*(w_xr-msl_x[i])+(w_yr-msl_y[i])*(w_yr-msl_y[i])
                kyori_r=math.sqrt(kyori_r)
                if kyori_r < 100:
                    #雲の上に画像を表示
                    msl_f[i]=False
                    # disp_img(screen, img_receive, w_xr, w_yr, w_theta, 0.25)
                    w_hp-=1
        kyori_e=(e_msl_x-w_xr)*(e_msl_x-w_xr)+(e_msl_y-w_yr)*(e_msl_y-w_yr)
        kyori_e=math.sqrt(kyori_e)
        if kyori_e < 100:
                    #雲の上に画像を表示
                    e_msl_f=False
                    # disp_img(screen, img_receive, w_xr, w_yr, w_theta, 0.25)
                    w_hp-=1

def contact_wall(): #雲とロボットの接触
    global xr,yr,w_xr,w_yr, w_hp,w_size,w_theta,v,omega,w_f 
    w_width=img_wall.get_width()/2*w_size
    w_height=img_wall.get_height()/2*w_size
    if w_theta==0:
        w_height=img_wall.get_width()/2*w_size
        w_width=img_wall.get_height()/2*w_size
    xkyori=xr-w_xr
    ykyori=yr-w_yr
    if w_hp>0:
        if w_f==4 and 0<xkyori<=w_width and -w_height<=ykyori<=w_height: #画面の左端から出てきた雲の右面に接触
            xr=w_xr+w_width
        if w_f==3 and -w_width<=xkyori<0 and -w_height<=ykyori<=w_height: #画面の右端から出てきた雲の左面に接触
            xr=w_xr-w_width
        if w_f==2 and -w_width<=xkyori<=w_width and 0<ykyori<=w_height: #画面の下端から出てきた雲の上面に接触
            yr=w_yr+w_height
        if w_f==1 and -w_width<=xkyori<=w_width and -w_height<=ykyori<0: #画面の上端から出てきた雲の下面に接触
            yr=w_yr-w_height

##############################################   メイン  ########################################################## 
def main():
    global xr, yr, vx, vy, theta, v, omega, deltaT, timer
    global idx, key_spc, joy_b, bgm, scorefile, score
    global se_shot, se_hit, se_hit3, written, zenkaiscore, switch
    pygame.init()
    pygame.joystick.init()
    pygame.display.set_caption("シューティングゲーム")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))  # ウィンドウのサイズ
    clock = pygame.time.Clock()  # 画面更新のためにクロックをつくっておく
    se_shot=pygame.mixer.Sound('./sound/mss.wav')
    se_hit=pygame.mixer.Sound('./sound/rep.wav')
    se_hit3=pygame.mixer.Sound('./sound/jan.wav')
    font = pygame.font.Font(None, 50) # 文字のフォントと大きさ
    font2 = pygame.font.Font(None, 140) # 文字のフォントと大きさ
    font3 = pygame.font.Font(None, 190) # 文字のフォントと大きさ
    pygame.mixer.music.load('./sound/bgm.wav')
    time_limit=30 #制限時間

    while True:
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  # ×が押されたら終了する
                pygame.quit()
                sys.exit()
        screen.fill((55, 55, 240))   # ウィンドウの内部に色を塗る
        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            joy_lr = joystick.get_axis(0)
            joy_ud = joystick.get_axis(1)
            joyL_lr = joystick.get_axis(2)
            joyL_ud = joystick.get_axis(3)
            jbtn1 = joystick.get_button(0)+joystick.get_button(1)+joystick.get_button(2)+joystick.get_button(3)
            jbtn2 = joystick.get_button(2) # Xボタン
            jbtn3 = joystick.get_button(4) # LB
            jbtn4 = joystick.get_button(5) # RB
            jbtn5 = joystick.get_button(7) # START
            jbtn6 = joystick.get_button(6)  # BACK
            jbtn7 = joystick.get_button(3)  # 黄色
            Joys=1
        except:
            Joys=0
        key=pygame.key.get_pressed()  # キーボードが押されたらそのキーをkeyとして記憶

        if idx==0:  # スタート
            screen.blit(img_start, [0, 0])
            if Joys == 1:  ###ゲームパッド
                sur = font.render('Press START button', True, (120,120,40)) # 色を指定して文字str(tmr)を画像surに置き換える
                disp_str(screen, sur, 0, 250, 0, 1.0)
                # screen.blit(sur, [0, 0])  # 座標を指定して画像を表示する
                if (key[pygame.K_SPACE]==True) or (jbtn5 !=0):
                    idx=1
                    # pygame.mixer.music.play(-1)
            else:  ###キーボード
                sur = font.render('Press SPACE key', True, (120,120,40)) # 色を指定して文字str(tmr)を画像surに置き換える
                disp_str(screen, sur, 0, 250, 0, 1.0)
                # screen.blit(sur, [0, 0])  # 座標を指定して画像を表示する
                if key[pygame.K_SPACE]==True:
                    idx=1
                    # pygame.mixer.music.play(-1)

        if idx==1:   # ゲームプレイ中
            timer=timer+1
            screen.blit(img_haikei0, [0, 0])
            disp_img(screen, img_haikei, 0, 0, timer*0.002, 1.0)  # 背景の画像を表示


            # # 地上局のメッセージの送信   
            # timing=timer%550      
            # # print(timing)    
            # if timing == 1:
            #     e_set_message() #メッセージの送信準備
            e_move_message(screen) #送信されたメッセージの移動と画像表示

            #雲の移動
            move_wall(screen,timer)
            contact_wall()

            ##### メッセージの送信 #######
            if Joys == 1:
                joy_b = (joy_b+1)*jbtn1
                key_spc = (key_spc+1)*key[pygame.K_SPACE]
            else:
                key_spc = (key_spc+1)*key[pygame.K_SPACE]
            if (joy_b%10 == 1 or key_spc%10 == 1):
                set_message()
                se_shot.play()
            move_message(screen) #送信されたメッセージの移動と画像表示

            ####### 人工衛星の制御
            if Joys == 1:
                if jbtn4 != 0:  # RB
                    omega =  - 1.0
                if jbtn3 != 0:  # LB
                    omega =   1.0
                if joy_ud < -0.01:
                    v =  70
                    # omega = 0
                if joy_ud > 0.01:
                    v = -70
                    # omega = 0
                if joyL_ud < -0.01:
                    v =  70
                    # omega = 0
                if joyL_ud > 0.01:
                    v = -70
                    # omega = 0
                if joy_lr  > 0.01:
                    omega =  - 1.0
                if joy_lr < -0.01:
                    omega =   1.0
            #####  キーボード矢印キーでの操作
            if key[pygame.K_RIGHT]==True:
                    omega = - 1
            if key[pygame.K_LEFT]==True:
                    omega =   1
            if key[pygame.K_UP]==True:
                    v=  60
                    omega=0
            if key[pygame.K_DOWN]==True:
                    v=- 60
                    omega=0
            #### 人工衛星の停止       
            if Joys == 1:
                if (jbtn2 != 0) or (jbtn6 != 0): # ボタンXまたはBACKが押されたら
                    omega = 0
                    v=0
            if key[pygame.K_s]==True:  # キーボードでs
                    v=0
                    omega=0 
            move_satellite(screen)  # 人工衛星の移動
            move_beni(screen)  # 地上局の移動
            time=timer*deltaT
            time=math.floor(time) #小数点以下切り捨て
            sur = font.render('Time: '+str(time_limit-time), True, (120,120,40)) # 色を指定して文字str(tmr)を画像surに置き換える
            screen.blit(sur, [0, 0])  # 座標を指定して画像を表示する
            sur = font.render('Score: '+str(score), True, (255,255,255)) # 色を指定して文字str(tmr)を画像surに置き換える
            screen.blit(sur, [0, 30])  # 座標を指定して画像を表示する
            if (time_limit-time) < 10:
                if switch == 0:
                    pygame.mixer.music.stop()
                    pygame.mixer.music.load('./sound/bgm.wav')
                    # pygame.mixer.music.play(-1)
                    switch=1
            if (time_limit-time) < 0:
                idx=2

        if idx==2:   # GAMEOVER表示   
            pygame.mixer.music.stop()
            # 最終スコアの表示
            if zenkaiscore < score:
                screen.blit(img_high, [0, 0])
                sur = font3.render(str(score), True, (255,200,255)) # 色を指定して文字str(tmr)を画像surに置き換える
                disp_str(screen, sur, 10, -45, 0, 1.0)
                # screen.blit(sur, [350, 250])  # 座標を指定して画像を表示する
                if written == 0:
                    scorefile.write(str(score))
                    written=1
            else:
                screen.blit(img_gameover, [0, 0])
                sur = font2.render('Score: '+str(score), True, (255,200,255)) # 色を指定して文字str(tmr)を画像surに置き換える
                disp_str(screen, sur, 0, -45, 0, 1.0)
                # screen.blit(sur, [105, 300])  # 座標を指定して画像を表示する
                if written == 0:
                    scorefile.write(str(zenkaiscore))
                    written=1
            scorefile.close()

        pygame.display.update()  
        clock.tick(30)  # 1秒間に30回，画面を更新

if __name__ == '__main__':
    main()