#include "acs_mag_ctrl.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <GL/glut.h>
#include <string.h>
#define Pi 3.1415926535
FILE *outputfile;   // データファイル

#define CTRL_DT_SEC              (1.0f)    
#define DETUMBLE_K_BDOT          (500.0f)
#define DETUMBLE_W_THRESH_DEG    (0.7f)
#define DETUMBLE_OK_COUNT        (60U)     
#define SUN_KP                   (1.0f)    
#define SUN_KD                   (0.1f)    

//------------------目標姿勢の設定--------------------
double theta_Rd    =  0.0 * Pi/180.0;
double axis_Rd[3]={0.0, 0.0, 1.0}; // 目標姿勢を決める角度と回転軸
double theta_init    =  90.0 * Pi/180.0;
double axis_init[3]={1.0, 1.0, 1.0}; 


unsigned int   imageDt = 10;           // アニメーションの画像更新インターバル[msec]
double         DeltaT=0.01; // シミュレーションの刻み時間
double         SimulationTime = 390.0; // シミュレーションの最大時間[sec]
double         Time=0.0;               // 経過時間
int icount_100=0;  // 100回に1回の処理用カウンタ
int icount_60=0;  // 60回に1回の処理用カウンタ
int Time_sec=0;	// 経過時間[sec]
int Time_min=0;	// 経過時間[min]
int control_flag=0; // 制御ON/OFFフラグ
int control_mode=2; // 制御モード 0:磁気固定 1:bdot制御 2:feedbak制御
int cal_flag=1; // 制御トルク計算済み


int WINDOW_W=1200, WINDOW_H= 700; 
int xbn = 0, ybn = 0, mB;
float dist, tW, eL, aZ;

double  e0[3]={1.0, 0.0, 0.0};
double  e1[3]={0.0, 1.0, 0.0};
double  e2[3]={0.0, 0.0, 1.0};

double	Kp[3][3]={0.01 , 0.0 , 0.0,
		  0.0 , 0.02 , 0.0,
		  0.0 , 0.0 , 0.03};
double	Kv[3][3]={2.0 , 0.0 , 0.0,
		  0.0 , 5.0 , 0.0,
		  0.0 , 0.0 , 7.0};
  
double	R[3][3];
double	Rd[3][3];
double  u[3]={0.0, 0.0,0.0};
double  a[3]={1.0, 2.0, 3.0};
double	omega[3]={0.0, 0.0, 0.0}; //  初期値は initialsetting で与えている．
double	J[3][3];
double	I[3][3]={1.0,0.0,0.0,
		 0.0,1.0,0.0,
		 0.0,0.0,1.0};      // 単位行列

double  TH[10000000],AH[3][10000000];
int     index_D=0;
double  Error=0.0;

double  omegaD[3] = {0.0, 0.0, 0.0}; // ωドット
double  RDot[3][3]; // Rドット
double  K_Omega[3]; // コントローラのΩ

double  R0i[3][3];  // 初期姿勢固定
double  theta_p = 0.0;  // 現在角度
// ファイルを表す変数を定義
FILE *outputfile;


extern uint8_t BSP_Option_Data[16];
static void get_sun_vector_body(float s[3]);
static float vec_norm3(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

#if false
static void vec_normalize(float v[3])
{
    float n = vec_norm3(v);
    if (n < 1e-8f) return;
    v[0] /= n; v[1] /= n; v[2] /= n;
}
#endif

void acs_mag_ctrl_init(acs_mag_ctrl_state_t *st)
{
    st->phase = ACS_MAG_PHASE_DETUMBLE;
    st->detumble_ok_counter = 0;
    st->duty_counter = 0;

}

bool acs_mag_ctrl_is_finished_detumble(const acs_mag_ctrl_state_t *st)
{
    return (st->phase != ACS_MAG_PHASE_DETUMBLE);
}

/* B-dot  */
// static void detumble_step(acs_mag_ctrl_state_t *st)
// {
//     DATA_CACHE_SENSOR_MAG_PRIMARY_BODY_FRAME_t mag;
//     DATA_CACHE_SENSOR_GYRO_BODY_FRAME_t        gyro;

//     dc_get_sensor_mag_primary_body_frame_data(&mag);
//     dc_get_sensor_gyro_body_frame_data(&gyro);
    
    
// 	gyro.dGyro_z = 0.0f;

//     /* B-dot (B_now - B_prev) / dt */
//     float dBx = (float)(mag.dMag_x_current - mag.dMag_x_previous);
//     float dBy = (float)(mag.dMag_y_current - mag.dMag_y_previous);
//     float dBz = (float)(mag.dMag_z_current - mag.dMag_z_previous);

//     float m_cmd[3];
//     m_cmd[0] = -DETUMBLE_K_BDOT * dBx / CTRL_DT_SEC;
//     m_cmd[1] = -DETUMBLE_K_BDOT * dBy / CTRL_DT_SEC;
//     m_cmd[2] = -DETUMBLE_K_BDOT * dBz / CTRL_DT_SEC;

//     /* MTQ */
//     int8_t mtq_x = (int8_t)m_cmd[0];
//     int8_t mtq_y = (int8_t)m_cmd[1];
//     int8_t mtq_z = (int8_t)m_cmd[2];

//     if (mtq_x > 100) mtq_x = 100;
//     if (mtq_x < -100) mtq_x = -100;
//     if (mtq_y > 100) mtq_y = 100;
//     if (mtq_y < -100) mtq_y = -100;
//     if (mtq_z > 100) mtq_z = 100;
//     if (mtq_z < -100) mtq_z = -100;

//     // Override Option Control
//     if(BSP_Option_Data[0] != 0) mtq_x = (int8_t)BSP_Option_Data[0];
//     if(BSP_Option_Data[1] != 0) mtq_y = (int8_t)BSP_Option_Data[1];
//     if(BSP_Option_Data[2] != 0) mtq_z = (int8_t)BSP_Option_Data[2];
    
//     acs_set_manual_mtq_control_mtq_frame(mtq_x, mtq_y, mtq_z);

//     /* */
//     float wx_deg = gyro.dGyro_x * (180.0f / 3.14159265f);
//     float wy_deg = gyro.dGyro_y * (180.0f / 3.14159265f);
//     float wz_deg = gyro.dGyro_z * (180.0f / 3.14159265f);
//     float w_norm = vec_norm3((float[3]){wx_deg, wy_deg, wz_deg});

//     if (w_norm < DETUMBLE_W_THRESH_DEG)
//     {
//         if (st->detumble_ok_counter < 0xFFFFFFFFU)
//             st->detumble_ok_counter++;
//     }
//     else
//     {
//         st->detumble_ok_counter = 0;
//     }

//     if (st->detumble_ok_counter >= DETUMBLE_OK_COUNT)
//     {
//         /* */
//         st->phase = ACS_MAG_PHASE_SUN_POINTING;
//         mtq_x = 0;
//         mtq_y = 0;
//         mtq_z = 0;
//         // Override Option Control
//         if(BSP_Option_Data[0] != 0) mtq_x = (int8_t)BSP_Option_Data[0];
//         if(BSP_Option_Data[1] != 0) mtq_y = (int8_t)BSP_Option_Data[1];
//         if(BSP_Option_Data[2] != 0) mtq_z = (int8_t)BSP_Option_Data[2];

//         acs_set_manual_mtq_control_mtq_frame(mtq_x, mtq_y, mtq_z);
//     }
// }


// static void get_sun_vector_body(float s[3])
// {
//     DATA_CACHE_SENSOR_COARSE_SUN_BODY_FRAME_t css;
//     dc_get_sensor_coarse_sun_body_frame_data(&css);

//     /* */
//     float sx_p = (float)css.i32Css_axis_x_plus;
//     float sx_m = (float)css.i32Css_axis_x_minus;
//     float sy_p = (float)css.i32Css_axis_y_plus;
//     float sy_m = (float)css.i32Css_axis_y_minus;
//     float sz_p = (float)css.i32Css_axis_z_plus;   // Z-

//     /* X, Y */
//     float vx = sx_p - sx_m;
//     float vy = sy_p - sy_m;

//     /* Z */
//     float vz = sz_p;

//     s[0] = vx;
//     s[1] = vy;
//     s[2] = vz;

    
//     float n = sqrtf(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
//     if (n < 1e-8f) {
//         s[0] = s[1] = s[2] = 0.0f;
//         return;
//     }
//     s[0] /= n;
//     s[1] /= n;
//     s[2] /= n;
// }

// static void sun_pointing_step(acs_mag_ctrl_state_t *st)
// {
//     (void)st;

//     float s[3];
//     get_sun_vector_body(s);

//     DATA_CACHE_SENSOR_GYRO_BODY_FRAME_t       gyro;
//     DATA_CACHE_SENSOR_MAG_PRIMARY_BODY_FRAME_t mag;

//     dc_get_sensor_gyro_body_frame_data(&gyro);
//     dc_get_sensor_mag_primary_body_frame_data(&mag);

    
// 	gyro.dGyro_z = 0.0f;
    
    
//     float z_body[3] = {0.0f, 0.0f, 1.0f};

    
//     float e[3] = {
//         z_body[1]*s[2] - z_body[2]*s[1],
//         z_body[2]*s[0] - z_body[0]*s[2],
//         z_body[0]*s[1] - z_body[1]*s[0]
//     };

//     float w[3] = {
//         (float)gyro.dGyro_x,
//         (float)gyro.dGyro_y,
//         (float)gyro.dGyro_z
//     };

    
//     float tau[3];
//     tau[0] = SUN_KP * e[0] - SUN_KD * w[0];
//     tau[1] = SUN_KP * e[1] - SUN_KD * w[1];
//     tau[2] = SUN_KP * e[2] - SUN_KD * w[2];

    
//     float B[3] = {
//         (float)mag.dMag_x_current,
//         (float)mag.dMag_y_current,
//         (float)mag.dMag_z_current
//     };
//     float B2 = B[0]*B[0] + B[1]*B[1] + B[2]*B[2];
//     if (B2 < 1e-10f) {
//         acs_set_manual_mtq_control_mtq_frame(0, 0, 0);
//         return;
//     }

    
//     float m_cmd[3];
//     m_cmd[0] = (B[1]*tau[2] - B[2]*tau[1]) / B2;
//     m_cmd[1] = (B[2]*tau[0] - B[0]*tau[2]) / B2;
//     m_cmd[2] = (B[0]*tau[1] - B[1]*tau[0]) / B2;

//     int8_t mtq_x = (int8_t)(m_cmd[0] * 1e6f);
//     int8_t mtq_y = (int8_t)(m_cmd[1] * 1e6f);
//     int8_t mtq_z = (int8_t)(m_cmd[2] * 1e6f);

//     if (mtq_x > 100) mtq_x = 100;
//     if (mtq_x < -100) mtq_x = -100;
//     if (mtq_y > 100) mtq_y = 100;
//     if (mtq_y < -100) mtq_y = -100;
//     if (mtq_z > 100) mtq_z = 100;
//     if (mtq_z < -100) mtq_z = -100;

//     // Override Option Control
//     if(BSP_Option_Data[0] != 0) mtq_x = (int8_t)BSP_Option_Data[0];
//     if(BSP_Option_Data[1] != 0) mtq_y = (int8_t)BSP_Option_Data[1];
//     if(BSP_Option_Data[2] != 0) mtq_z = (int8_t)BSP_Option_Data[2];
    
    
//     acs_set_manual_mtq_control_mtq_frame(mtq_x, mtq_y, mtq_z);
// }

// void acs_mag_ctrl_step(acs_mag_ctrl_state_t *st)
// {
//     st->duty_counter++;

//     if (st->duty_counter >= 60)
//         st->duty_counter = 0;

//     // 電源低下時にはトルカを停止する
//     if(EPS_2nd_Battery() < 15.0f)
//     {
//         int8_t mtq_x = (int8_t)(0);
//         int8_t mtq_y = (int8_t)(0);
//         int8_t mtq_z = (int8_t)(0);
//         acs_set_manual_mtq_control_mtq_frame(mtq_x, mtq_y, mtq_z);
//         sds_sens_release(SDS_AVAILABLESENSORS_SENSOR_MAG_PRIMARY);
//         return;
//     }

//     if (st->duty_counter == 0)
//     {
//         int8_t mtq_x = (int8_t)(0);
//         int8_t mtq_y = (int8_t)(0);
//         int8_t mtq_z = (int8_t)(0);

//         // Override Option Control
//         if(BSP_Option_Data[0] != 0) mtq_x = (int8_t)BSP_Option_Data[0];
//         if(BSP_Option_Data[1] != 0) mtq_y = (int8_t)BSP_Option_Data[1];
//         if(BSP_Option_Data[2] != 0) mtq_z = (int8_t)BSP_Option_Data[2];

//         acs_set_manual_mtq_control_mtq_frame(mtq_x, mtq_y, mtq_z);
//         sds_sens_release(SDS_AVAILABLESENSORS_SENSOR_MAG_PRIMARY);
//         return;
//     }

    
//     if (st->duty_counter == 55)
//     {
//         sds_sens_lock(SDS_AVAILABLESENSORS_SENSOR_MAG_PRIMARY);
//     }

    
//     if (st->duty_counter >= 55)
//     {
//         switch (st->phase)
//         {
//             case ACS_MAG_PHASE_DETUMBLE:
//                 detumble_step(st);
//                 break;

//             case ACS_MAG_PHASE_SUN_POINTING:
//                 sun_pointing_step(st);
//                 break;

//             default:
//                 st->phase = ACS_MAG_PHASE_DETUMBLE;
//                 break;
//         }
//     }
// }

//---------------------------------------------------
//----------------------キーボードの押し下げに対する処理----------------------
void myKeyboard(unsigned char key, int x, int y)
{
        switch (key) {

		case 27:        exit(0);  //ESCキーで終了
				break;

	}

}

void specialKey(int key, int x, int y)
{
        switch (key) {  
                case GLUT_KEY_UP: // ↑キーの場合+0.01
                break;
                case GLUT_KEY_DOWN: // ↓キーの場合-0.01
                break;
                case GLUT_KEY_RIGHT: // →キーの場合
                break;
                case GLUT_KEY_LEFT: // ←キーの場合
                break;
        }
}
//-------------------------タイマー（0.05秒に１回呼び出される）-----------------
void myTimer(int value)
{
        if (value == 1) 
        {
                glutTimerFunc(imageDt, myTimer, 1);

        	if (Time > SimulationTime){
        		exit(0);
        	}
                glutPostRedisplay();
        }
}


//------------------------------------------------------------------------------
void myMouse( int button, int state, int x, int y )
{
    if (state == GLUT_DOWN) {
        mB = button;
        xbn = x;   ybn = y;
    }
}

//--------------------------------------------------------------------------------
void myMotion(int x, int y)
{
    int xD, yD;

    xD = x - xbn;    yD = y - ybn;
    switch(mB){
    case GLUT_LEFT_BUTTON:
        aZ -= (float) xD/2.0;  eL -= (float) yD/2.0;
        break;
    case GLUT_MIDDLE_BUTTON:
        tW = fmod (tW + xD, 360.0);
        break;
    case GLUT_RIGHT_BUTTON:
        dist -= (float) yD/40.0;
        break;
    }
    xbn = x;    ybn = y;
    glutPostRedisplay();
}

//-----------------------------------------------------------------------------------
void p_view( void )
{
    glTranslatef( 0.0, 0.0, -dist);
    glRotatef( -tW, 0.0, 0.0, 1.0);
    glRotatef( -eL, 1.0, 0.0, 0.0);
    glRotatef( -aZ, 0.0, 1.0, 0.0);
}


//--------------------R, G, B で色を指定----------------------------------------
void setcolor(float R, float G, float B)
{
        float ambient[4]; 
        float diffuse[4]; 
        float specular[4]; 
        float shininess[] = {0.6};
        ambient[0]=R;  ambient[1]=G;  ambient[2]=B;  ambient[3]=1.0;  
        diffuse[0]=R;  diffuse[1]=G;  diffuse[2]=B;  diffuse[3]=1.0;  
        specular[0]=R; specular[1]=G; specular[2]=B; specular[3]=1.0;  
        glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,   diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR,  specular);
        glMaterialfv(GL_FRONT, GL_SHININESS, shininess);      
}

//--------------------金属的な色の指定----------------------------------------
void setcolor2(   GLfloat ambr, GLfloat ambg, GLfloat ambb,
   GLfloat difr, GLfloat difg, GLfloat difb,
   GLfloat specr, GLfloat specg, GLfloat specb, GLfloat shine)
{
   GLfloat mat[4];
   mat[0] = ambr; mat[1] = ambg; mat[2] = ambb; mat[3] = 1.0;
   glMaterialfv(GL_FRONT, GL_AMBIENT, mat);
   mat[0] = difr; mat[1] = difg; mat[2] = difb;
   glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);
   mat[0] = specr; mat[1] = specg; mat[2] = specb;
   glMaterialfv(GL_FRONT, GL_SPECULAR, mat);
   glMaterialf(GL_FRONT, GL_SHININESS, shine * 128.0);
   
}


//----------------------- xyz軸を描く---------------------------------------------------
void axis(void)
{
	glLineWidth(1.0);
		glBegin(GL_LINES);
		glColor3d(1.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.25, 0.0, 0.0);
	glEnd();
	glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 0.25, 0.0);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, 0.25);
	glEnd();
}

void Cube()
{
        glPushMatrix();
                glScaled(0.3, 0.3, 0.3);        // スケールの変換
				if(control_flag==0){
                	setcolor(0.3, 0.3, 0.3); // 制御していないときグレー
				}else{
					setcolor(0.0, 0.8, 0.0); // 制御しているときグリーン
				}
                glutSolidCube(1.0);     //  基礎となる物体
        glPopMatrix();
}

//---------------------------------------------------------------------
void arrow(double length)  // 原点からx方向に長さlengthの矢印を描く
{
		double a=0.03;  // 矢の先端の長さ
		glPushMatrix();
		glTranslated(length-a, 0.0, 0.0);
		glRotated(90.0, 0.0, 1.0, 0.0);
		glutSolidCone(0.05, 0.07, 10, 10);
		glPopMatrix();
		glLineWidth(3.0);     
        glBegin(GL_LINES);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(length, 0.0, 0.0);
        glEnd();

}

//-------------(x1, y1, z1)から(x2, y2, z2)へ向かうベクトルを矢印で描く------
void vector(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double x, y, z;
	double length, gamma, beta;
	x=x2-x1;	y=y2-y1;	z=z2-z1;
	gamma=atan2(y, sqrt(x*x+z*z)); 	beta=-atan2(z, x);
	glPushMatrix();
	glTranslated(x1, y1, z1);
	glRotated(beta*180.0/Pi, 0.0, 1.0, 0.0);
	glRotated(gamma*180.0/Pi, 0.0, 0.0, 1.0);
	arrow(sqrt(x*x+y*y+z*z));
	glPopMatrix();
}


//-----------------------------------------------------------------------------
/* Function which can be used like printf() of C language.
 * The position of beginning write of the character is (x,y).
 */ 
void myprintf(int x, int y, char *aFmt, ...){
	int  i;
	char buf[1024];
	va_list ap;					// defined in <stdarg.h>

	glPushMatrix();
	glLoadIdentity();
	va_start(ap, aFmt);
	vsprintf(buf, aFmt, ap);	
	glRasterPos3d(0,0,-1);
	glBitmap(0, 0, 0, 0, (int)x, (int)y, NULL);  
	for(i = 0; i < strlen(buf); i++){
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, buf[i]);
	}
	va_end(ap);
	glPopMatrix();
}

//---------------------------------------------------
//--------------------スカラとベクトルの積--------------------------
void StimesV(double a, double v[], double vout[])
{
	int i;
	for(i=0; i<3; i++){
		vout[i] = a*v[i];
	}
}

//---------------------ベクトルとベクトルの外積----------------------------------

void VprodV(double va[], double vb[], double vout[])
{
	vout[0]=va[1]*vb[2]-va[2]*vb[1];
	vout[1]=va[2]*vb[0]-va[0]*vb[2];
	vout[2]=va[0]*vb[1]-va[1]*vb[0];
}
//---------------------ベクトルとベクトルの和----------------------------------

void VplusV(double va[], double vb[], double vout[])
{
	vout[0]= va[0] + vb[0];
	vout[1]= va[1] + vb[1];
	vout[2]= va[2] + vb[2];
}
//----------------------行列とベクトルの積---------------------------------

void MtimesV(double M[][3], double v[], double vout[])
{
	int i, j;
	for(i=0; i<3; i++){
		vout[i]=0.0;
		for(j=0; j<3; j++){
			vout[i] += M[i][j]*v[j];
		}
	}
}


//----------------------ベクトルから交代行列をつくる---------------------------------
void VtoS(double v[], double Mout[][3])
{
	int i,j;
	Mout[0][0]=0.0;   Mout[0][1]=-v[2];  Mout[0][2]=v[1];
	Mout[1][0]=v[2];  Mout[1][1]=0.0;    Mout[1][2]=-v[0];
	Mout[2][0]=-v[1]; Mout[2][1]=v[0];   Mout[2][2]=0.0;
}
//----------------------スカラと行列の積---------------------------------
void StimesM(double a, double M[][3], double Mout[][3])
{
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			Mout[i][j] = a * M[i][j];
		}
	}
}
//-----------------------行列と行列の積--------------------------------
void MtimesM(double Ma[][3], double Mb[][3], double Mout[][3])
{
	int i, j, k;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			Mout[i][j]=0.0;
			for(k=0; k<3; k++){
				Mout[i][j] += Ma[i][k]*Mb[k][j];
			}
		}
	}
}
//------------------------行列と行列の和-------------------------------
void MplusM(double Ma[][3], double Mb[][3], double Mout[][3])
{
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			Mout[i][j] = Ma[i][j] + Mb[i][j];
		}
	}
}
//------------------------行列の転置行列-------------------------------
void MtransM(double Ma[][3], double Mout[][3])
{
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			Mout[i][j] = Ma[j][i];
		}
	}
}
//------------------------行列のコピー------------------------------
void McopyM(double Ma[][3], double Mout[][3])
{
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			Mout[i][j] = Ma[i][j];
		}
	}
}
//------------------------ベクトルのコピー------------------------------
void VcopyV(double v[], double vout[])
{
	int i;
	for(i=0; i<3; i++){
		vout[i]=v[i];
	}
}
//-------------------------------------------------------
//   回転行列Rから回転角と回転軸を求める
void RtoTheta(double R[][3], double *the, double a[])
{
	double cs, si, ax, ay, az, taa;
	cs=(R[0][0]+R[1][1]+R[2][2]-1.0)/2.0; // まずcos（コサイン）を求める
	if(fabs(cs) >=  1.0){
		if(cs < 0) cs=-0.999999;
		if(cs > 0) cs= 0.999999;
	}
	si=sqrt(1.0-(cs*cs)); // sin（サイン）を求める
	*the=atan2(si,cs);    // sinとcosからアークタンジェントで回転角(0以外）を求める
	// 以下は回転軸ベクトルaを求める計算
	ax=(1/(2*si))*(R[2][1]-R[1][2]);
	ay=(1/(2*si))*(R[0][2]-R[2][0]);
	az=(1/(2*si))*(R[1][0]-R[0][1]);
	taa=sqrt(ax*ax+ay*ay+az*az);
	if(fabs(taa) < 0.00000001 ){
		printf("回転軸が計算できません\n");
	}
	a[0]=ax/taa;
	a[1]=ay/taa;
	a[2]=az/taa;  // 回転軸ベクトル a(t) （単位ベクトル）
}

//----------------------------------------------------------------------
//   回転角と回転軸から回転行列Rを求める
void ThetatoR(double *the, double a[], double R[][3])
{
	double taa, A[3][3], Ma0[3][3], Ma1[3][3], Ma2[3][3];
	taa=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);  // ベクトルa の長さ
	a[0]=a[0]/taa;
	a[1]=a[1]/taa;
	a[2]=a[2]/taa;  // 回転軸の単位ベクトル a を求めた
	A[0][0]=0.0;  A[0][1]=-a[2];A[0][2]=a[1];
	A[1][0]=a[2]; A[1][1]=0.0;  A[1][2]=-a[0];
	A[2][0]=-a[1];A[2][1]=a[0];A[2][2]=0.0; // 歪対称行列
	StimesM(sin(*the), A, Ma0);
	MplusM(I, Ma0, Ma1);
	MtimesM(A, A, Ma0);
	StimesM(1-cos(*the), Ma0, Ma2);
	MplusM(Ma1, Ma2, R);         
}

//---------------------- 初期値設定（時刻 t=0 のときの値）
void initialsetting(void)
{
	int i, j, k, l;
	double Ma0[3][3], Ma1[3][3], Ma2[3][3], Ma3[3][3];
	double V0[3], V1[3], V2[3], V3[3], V30[3], V31[3], V32[3];
	double a_a[3];
	// 慣性テンソル
	J[0][0]=3.0;  J[0][1]=0.0;  J[0][2]=0.0;
	J[1][0]=0.0;  J[1][1]=4.0;  J[1][2]=0.0;
	J[2][0]=0.0;  J[2][1]=0.0;  J[2][2]=5.0;
	//  Rd（目標姿勢） 
	ThetatoR(&theta_Rd, axis_Rd, Rd);    // 初期角度と回転軸から回転行列（初期値）を求める
 	omega[0]=0.0;
	omega[1]=0.0;
	omega[2]=0.0;             // ω(t)
	ThetatoR(&theta_init, axis_init, R);    // 回転行列を求める
	McopyM(R,R0i); // 初期姿勢を保存
}

void eular_equation(double omega[], double omega_out[])	// オイラー方程式による角速度の更新-------
{
	double Ma0[3][3];
	double V0[3], V1[3], V2[3];
	MtimesV(J, omega, V0);  // Jω
	VprodV(V0, omega, V1);  // Jω×ω
	VplusV(V1, u, V2);  // Jω×ω + u
	Ma0[0][0]=1.0/3.0; Ma0[0][1]=0.0; Ma0[0][1]=0.0;
	Ma0[1][0]=0.0; Ma0[1][1]=1.0/4.0; Ma0[1][2]=0.0;
	Ma0[2][0]=0.0; Ma0[2][1]=0.0; Ma0[2][2]=1.0/5.0;  // J^-1 （慣性テンソルの逆行列）
	MtimesV(Ma0, V2, omega_out);  // ωドット
}

void cal_torque_mag(double u[])  // トルクの計算（磁気トルカ）
{
	double Ma0[3][3], Ma1[3][3], Ma2[3][3];
	double V0[3], V1[3], V2[3];

	// detumble_step(acs_mag_ctrl_state_t *st);
	StimesM(-1.0, Kv, Ma1); // -Kv
	MtimesV(Ma1, omega, V1); // -Kvω
	VcopyV(V1, u);  //  トルクu
}

void cal_torque_bdot(double u[])  // トルクの計算（Bdot制御）
{
	double Ma0[3][3], Ma1[3][3], Ma2[3][3];
	double V0[3], V1[3], V2[3];

	// detumble_step(acs_mag_ctrl_state_t *st);
	StimesM(-1.0, Kv, Ma1); // -Kv
	MtimesV(Ma1, omega, V1); // -Kvω
	VcopyV(V1, u);  //  トルクu
}

void cal_torque_feedback(double u[])  // トルクの計算（目標姿勢Rdに一致させるためのトルク）
{
	double Ma0[3][3], Ma1[3][3], Ma2[3][3];
	double V0[3], V1[3], V2[3], V3[3], V30[3], V31[3], V32[3];
	double VR1[3], VR2[3], VR3[3];
		StimesV(a[0], e0, V1);  // a_1e_1
		StimesV(a[1], e1, V2);  // a_2e_2
		StimesV(a[2], e2, V3);  // a_3e_3
		MtransM(Rd, Ma0);  // Rd^T
		MtimesM(Ma0, R, Ma1);   // Rd^TR
		MtimesV(Ma1, e0, VR1);  // Rd^TRe_1
		MtimesV(Ma1, e1, VR2);  // Rd^TRe_2
		MtimesV(Ma1, e2, VR3);  // Rd^TRe_3
		VprodV(V1, VR1, V30);   // a_1e_1×Rd^TRe_1
		VprodV(V2, VR2, V31);   // a_2e_2×Rd^TRe_2
		VprodV(V3, VR3, V32);   // a_3e_3×Rd^TRe_3
		VplusV(V30, V31, V0);   // a_1e_1×Rd^TRe_1 + a_2e_2×Rd^TRe_2
		VplusV(V0, V32, K_Omega); // Ω= a_1e_1×Rd^TRe_1 + a_2e_2×Rd^TRe_2 + a_3e_3×Rd^TRe_3
		StimesM(-1.0, Kv, Ma1); // -Kv
		MtimesV(Ma1, omega, V1); // -Kvω
		StimesM(-1.0, Kp, Ma2);  // -Kp
		MtimesV(Ma2, K_Omega, V2); // -KpΩ
		VplusV(V1, V2, V0); // u=-Kvω - KpΩ
		VcopyV(V0, u);  //  トルクu
}

//----------------------各種変数の計算------------------------------
void calculation(void){ 
	int		i,j,k,l,mn;
	double		X1,Y1,Z1,X2,Y2,Z2,X3,Y3,Z3;
	double Ma0[3][3], Ma1[3][3], Ma2[3][3];
	double V0[3], V1[3], V2[3], V3[3], V30[3], V31[3], V32[3], at[3];
	double VR1[3], VR2[3], VR3[3];
	double theta_temp, a_temp[3];
	double norm_tau;
	double a_p[3];
	double the,th, cs;
	double k1[3], k2[3], k3[3], k4[3], omegaD[3];

	Time += DeltaT;              // 経過時間を更新
	icount_100++;  // 100ステップごとにファイルに書き込む
	if(icount_100 == 100){ // 1秒経過したときの増分とリセット
		icount_100=0;
		cal_flag=0;  // トルク計算のフラグをリセット
		Time_sec++;
		icount_60++;  
	}
	if(icount_60 == 60){  // 60秒=1分経過したときの増分とリセット
		icount_60=0;
		Time_min++;
	}
	if(icount_60 < 55){  // 制御のONOFFの判定（55秒から60秒まで制御）
		control_flag = 0;  // 制御OFF
	}else{
		control_flag = 1;  // 制御ON
	}

	// 時間による制御モードの切り替え（0:あえて振動させる, 1:Bdot制御, 2:目標姿勢への収束）
	// if( Time_min < 30){  // 30分間は制御モード0（あえて振動させる）
	// 	control_mode = 0;
	// }else if( Time_min < 300){  // 30分から300分までは制御モード1（Bdot制御）
	// 	control_mode = 1;
	// }else{  // 300分以降は制御モード2（目標姿勢への収束）
	// 	control_mode = 2;
	// }

	// omega を使って R を更新
	VtoS(omega, Ma0);  // ω(t)の交代行列 ω^(t)
	MtimesM(R, Ma0, RDot); // R(t)ω^(t)
	StimesM(DeltaT, RDot, Ma2); // ΔtR(t)ω^(t)
	MplusM(R, Ma2, Ma0); // R(t)+ΔtR(t)ω^(t)
	McopyM(Ma0, R);  // R(t+Δt)の更新
	RtoTheta(R, &theta_p, a_p); // 現在姿勢での角度theta_pと回転軸a_pを求める

	// オイラー方程式による角速度の更新（4次のルンゲクッタ法）--------------------
	eular_equation(omega, k1);
	StimesV(0.5*DeltaT, k1, V0);  // Δｔ×ωドット
	VplusV(omega, k1, V1);  //
	eular_equation(V1, k2);
	StimesV(0.5*DeltaT, k2, V0);
	eular_equation(V0, k3);
	StimesV(DeltaT, k3, V0);
	VplusV(omega, k3, V1);
	eular_equation(V1, k4);
	StimesV(DeltaT/6.0, k1, V0);
	StimesV(DeltaT/3.0, k2, V1);
	StimesV(DeltaT/3.0, k3, V2);
	StimesV(DeltaT/6.0, k4, V3);
	VplusV(V0, V1, V0);
	VplusV(V0, V2, V0);
	VplusV(V0, V3, V0);
	VplusV(omega, V0, V1);
	VcopyV(V1, omega);     // ω の更新


	// トルクの計算（55秒から1秒ごとに59秒までトルクuを更新）---------------------------
	if( (icount_60== 55 && cal_flag==0) || (icount_60== 56 && cal_flag==0) ||
	    (icount_60== 57 && cal_flag==0) || (icount_60== 58 && cal_flag==0) ||
	    (icount_60== 59 && cal_flag==0) ){
			// トルクuの計算は3つのモードから選択する
			if(control_mode==0){  
				cal_torque_mag(u);  // あえて振動させる
			}else if(control_mode==1){  
				cal_torque_bdot(u);  // デタンブリング（Bdot制御）
			}else{  
				cal_torque_feedback(u);  // 目標姿勢への収束
			}
			cal_flag=1;  // uを計算した後，1秒間はトルク計算をしないようにする
	}
	if(icount_60 < 55){  // 制御OFFのとき
			u[0]=0.0; u[1]=0.0; u[2]=0.0;  // 制御OFFのときはトルクをゼロにする	
	}

	// ファイルに書き込む
	if((Time_sec > 53) && (Time_sec < 62)){  
		fprintf(outputfile, "%lf, %lf, %lf, %lf, %4d, %lf, %lf, %lf\n", 
			omega[0], omega[1], omega[2], Time, Time_sec, u[0], u[1], u[2]);
	}
}

//-------------------------------座標軸---------------------------------------
void axis1()
{
	int i = 1; // mode によって変化
		glPushMatrix();
			setcolor(0.5+0.5*sin(  -0.5*i*Pi/6), 
                        	 	0.5+0.5*sin( -2*Pi/3 -0.5*i*Pi/6),
                         	 	0.5+0.5*sin( -4*Pi/3 -0.5*i*Pi/6));  // 色の指定
			vector(0.0,0,0.0, 0.7, 0 ,0);
		glPopMatrix();
		glPushMatrix();
        		setcolor(0.5+0.5*sin(  -0.5*i*Pi/2), 
                        	 	0.5+0.5*sin( -2*Pi/3 -0.5*i*Pi/2),
                         	 	0.5+0.5*sin( -4*Pi/3 -0.5*i*Pi/2));  // 色の指定
			vector(0.0,0,0.0, 0, 0.7 ,0);
		glPopMatrix();
		glPushMatrix();
        		setcolor(0.5+0.5*sin(  -0.5*i*Pi), 
                        	 	0.5+0.5*sin( -2*Pi/3 -0.5*i*Pi),
                         	 	0.5+0.5*sin( -4*Pi/3 -0.5*i*Pi));  // 色の指定
			vector(0.0,0,0.0, 0, 0 , 0.7);
		glPopMatrix();
}


//-----------------------固定したxyz軸-------------------------------
void axis_I()
{
	setcolor(0.3, 0.3, 0.3);
	vector(0.0,0.0,0.0,  1.2, 0.0, 0.0);
	vector(0.0,0.0,0.0,  0.0, 1.2, 0.0);
	vector(0.0,0.0,0.0,  0.0, 0.0, 1.2);
}


//--------------------回転行列Rで回転されたxyz軸--------------------
void Rdisp(double R[][3])
{
		setcolor(1.0, 0.0, 0.0);
		vector(0.0,0.0,0.0,    R[0][0], R[1][0], R[2][0]);
		setcolor(0.0, 1.0, 0.0);
		vector(0.0,0.0,0.0,    R[0][1], R[1][1], R[2][1]);
		setcolor(0.0, 0.0, 1.0);
		vector(0.0,0.0,0.0,    R[0][2], R[1][2], R[2][2]);
}


//-----------（左上）-----------------
void drawobject1()
{
	int i=5; 
	double theta_l, theta_t;
	double omegal, ox, oy, oz, Romega[3], al[3], Ma0[3][3], Ma1[3][3];
	double Rh[3][3], th, ah[3], thk, ahk[3];
	double ahA[3], thA;

	axis_I();

	// //  表示用の角度と回転軸のデータを配列THとAHに保存
	// TH[index_D]=thA;
	// AH[0][index_D]=ahA[0]; AH[1][index_D]=ahA[1]; AH[2][index_D]=ahA[2];
	// index_D++;
	// //  THとAHに入っているデータで初期から現在までの軌道を表示
	// for(i=0;i<index_D; i+=15 ){
	// glPushMatrix();
	// 	glRotated(TH[i]*180.0/Pi, AH[0][i], AH[1][i], AH[2][i]);
	// 	setcolor(0, 0, 1);
	// 	axis1();
	// glPopMatrix();
	// }

	Rdisp(R);    // 軌道上の姿勢
	RtoTheta(R, &thA, ahA);
	Error = thA*180.0/Pi;
	glPushMatrix();
		glRotated(thA*180.0/Pi, ahA[0], ahA[1], ahA[2]);
		Cube();
	glPopMatrix();  
}

//---------------（右上）-----------------
void drawobject2()
{
	int k;
	int i=5; 
	double theta_l, theta_t;
	double omegal, ox, oy, oz, Romega[3], al[3], Ma0[3][3], Ma1[3][3];
	double Rh[3][3], th, ah[3], thk, ahk[3], R0[3][3];
	axis_I();
	Rdisp(R0i);    // 初期姿勢
	RtoTheta(R0i, &th, ah);
	glPushMatrix();
		glRotated(th*180.0/Pi, ah[0], ah[1], ah[2]);
		Cube();
	glPopMatrix();
}


// //--------- -----（左中）-----------------
// void drawobject3()
// {
// 	int i=5; 
// 	double theta_l, theta_t;
// 	double omegal, ox, oy, oz, Romega[3], al[3], Ma0[3][3], Ma1[3][3];
// 	axis_I();
// }


//---------------（右中）-----------------
void drawobject4()
{
	int i=5; 
	double theta_l, theta_t;
	double omegal, ox, oy, oz, Romega[3], al[3], Ma0[3][3], Ma1[3][3];
	double Rh[3][3], th, ah[3], thk, ahk[3];
	axis_I();
	Rdisp(Rd);    // 目標姿勢
	// RtoTheta(Rd, &th, ah);
	// Error = th*180.0/Pi;
	glPushMatrix();
		glRotated(0.0, 0, 0, 1);
		Cube();
	glPopMatrix();  
}

//----------------------------------------------------------------------------------------

void myDisplay(void)
{
	int i, k;
	double the4, a4[3];
	double DZ=4.4;

	// 画面1回更新中に物理計算をそれよりも多く行う．
	for(k=0; k<5; k++){
		for(i=0; i<2; i++){
			calculation();  // 刻み時間DeltaT=0.01秒間ごとの物理量の更新
		}
		RtoTheta(R, &the4, a4);
		ThetatoR(&the4, a4, R);  // 回転行列の性質を回復
	}

	//------------------表示の設定-----------------------
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//--------- -----（左上）-----------------
	glViewport(0,100,900,600);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 900.0/600.0, 0.05, 15.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(2.8, 2.4, 2.0,  0.0, 0.0, 0.0,   0.0, 0.0, 1.0); // 視点の設定
	p_view();
	drawobject1();

	//---------------（右上）-----------------
	glViewport(900,400,300,300);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 300.0/300.0, 0.05, 15.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(2.8, 2.4, 2.0,  0.0, 0.0, 0.0,   0.0, 0.0, 1.0); // 視点の設定
	p_view();
	drawobject2();

	//---------------（右中）-----------------
	glViewport(900,100,300,300);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 300.0/300.0, 0.05, 15.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(2.8, 2.4, 2.0,  0.0, 0.0, 0.0,   0.0, 0.0, 1.0); // 視点の設定
	p_view();
	drawobject4();

	//---------------(下）--文字を表示-------------
	glViewport(0,0,1200,100);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 1200.0/100.0, 0.05, 15.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	setcolor(0.0, 0.0, 0.0);
	//-----------------------------------------------------------------------------------------------
	myprintf(-600, 130, "R[0][0]=%4.3lf , R[0][1]=%4.3lf , R[0][2]=%4.3lf",R[0][0],R[0][1],R[0][2]);
	myprintf(-600, 120, "R[1][0]=%4.3lf , R[1][1]=%4.3lf , R[1][2]=%4.3lf",R[1][0],R[1][1],R[1][2]);
	myprintf(-600, 110, "R[2][0]=%4.3lf , R[2][1]=%4.3lf , R[2][2]=%4.3lf",R[2][0],R[2][1],R[2][2]);
	myprintf(-600, 90, "omega[0]=%4.3lf , omega[1]=%4.3lf , omega[2]=%4.3lf",omega[0],omega[1],omega[2]);       // ω
	myprintf(-600, 50, "u[0]=%4.3lf , u[1]=%4.3lf , u[2]=%4.3lf",u[0],u[1],u[2]);
	myprintf(-600, 30, "Rd[0][0]=%4.3lf , Rd[0][1]=%4.3lf , Rd[0][2]=%4.3lf",Rd[0][0],Rd[0][1],Rd[0][2]);
	myprintf(-600, 20, "Rd[1][0]=%4.3lf , Rd[1][1]=%4.3lf , Rd[1][2]=%4.3lf",Rd[1][0],Rd[1][1],Rd[1][2]);
	myprintf(-600, 10, "Rd[2][0]=%4.3lf , Rd[2][1]=%4.3lf , Rd[2][2]=%4.3lf",Rd[2][0],Rd[2][1],Rd[2][2]);
	myprintf(-600, -10, "theta_p=%4.3lf", theta_p);
	myprintf(0, 190, "Time=%4.3f", Time);
	myprintf(0, 170, "Error=%4.3f", Error);
	myprintf(-600, 190, "icount_100=%4d", icount_100);
	myprintf(-450, 190, "Time_sec=%4d", Time_sec);
	myprintf(-300, 190, "icount_60=%4d", icount_60);
	myprintf(-150, 190, "Time_min=%4d", Time_min);
	//---------------------------------------------------
	glutSwapBuffers();  // 画面に映像を表示
}

//----------------  照明の設定  -----------------------------------
void mySetLight(void)
{
	GLfloat ambient[] = {0.0, 0.0, 0.0, 1.0};
	GLfloat diffuse[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat position[] = {0.0, 0.0, 4.0, 0.0};
	GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
	GLfloat local_view[] = {0.0};
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
	glFrontFace(GL_CW);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST); 
}

//------------------  初期処理  -------------------------------------
void myInit(char *progname)
{
	// ファイルを開く（この例ではファイル名は x.txt）
	outputfile = fopen("x.txt", "w");  // ファイルを書き込み用にオープン(開く)
 	if (outputfile == NULL) {          // オープンに失敗した場
 	   printf("cannot open\n");         // エラーメッセージを出して
 	   exit(1);                         // 異常終ao
 	}

    initialsetting();
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH); 
    glutInitWindowSize(WINDOW_W, WINDOW_H);// ウィンドウの幅と高さ
    glutInitWindowPosition(30, 10);         // ウィンドウの左上の位置
    glutCreateWindow("OpenGL");       
    glClearColor(1.0, 1.0, 1.0, 0.0);   //背景の色
}

//----------メイン---------------------------------
int main(int argc, char** argv)
{
        glutInit(&argc, argv);
        myInit(argv[0]);
        glutKeyboardFunc(myKeyboard);
        glutSpecialFunc(specialKey); 
        glutMouseFunc(myMouse);
        glutMotionFunc(myMotion);
        mySetLight();
        glEnable(GL_LIGHTING);  
        glEnable(GL_DEPTH_TEST);
        glutTimerFunc(imageDt, myTimer, 1);
        glutDisplayFunc(myDisplay);
        glutMainLoop();
		// ファイルのクローズ
		fclose(outputfile);
        return 0;
}
